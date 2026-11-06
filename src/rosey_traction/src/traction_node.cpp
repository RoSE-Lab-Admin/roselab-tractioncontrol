// traction_node: estimates per-wheel slip from motor current and scales the forward speed
// command. All per-cycle logic lives in TractionCore; this file is the ROS plumbing.
//
//   /cmd_vel (TwistStamped) ─▶ traction_node ─▶ /traction/cmd_vel ─▶ roseybot_base_controller
//   /dynamic_joint_states  ─┤                └▶ /traction/state (rosey_traction_msgs/TractionState)
//   imu_topic (Imu)        ─┘
//
// The output command is published on every timer tick from the latest input, restamped with the
// current time. The output goes to zero no later than cmd_timeout after the last input: the
// input counts as fresh only while it is at most (cmd_timeout - one timer period) old, so the
// last non-zero output is never older than cmd_timeout. Restamping means diff_drive's own
// cmd_vel_timeout no longer catches a silent input, so this check is the one that does.
//
// IMU convention. The UM7 publishes on imu_link, which is mounted aligned with base_link
// (URDF rpy 0 0 0), with the orientation converted to ENU (tf_ned_to_enu: true). In REP-103
// base_link has x forward, y left, z up, so ZYX Euler angles from the quaternion give
//   roll  = rotation about x,  pitch = rotation about y (positive pitch = nose DOWN).
// The normal load uses tilt_cos = cos(roll) * cos(pitch) = 1 - 2 (qx^2 + qy^2), the vertical
// component of the body z axis. On a pure incline this equals cos(pitch), and it does not
// depend on which axis is called pitch. That matters because um7_driver remaps the quaternion
// and swaps roll and pitch in its rpy output, and the mounting has not been checked on the
// bench yet. The reported pitch is configurable (pitch_axis: "pitch" or "roll", pitch_sign)
// until the Phase 4 tilt test settles it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosey_traction/traction_core.hpp"
#include "rosey_traction_msgs/msg/traction_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace rosey_traction
{

class TractionNode : public rclcpp::Node
{
public:
  TractionNode()
  : Node("traction_node")
  {
    const auto config = loadConfig();
    core_ = std::make_unique<TractionCore>(config);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    joint_sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states", sensor_qos,
      [this](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) {onJointStates(*msg);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {onCmd(*msg);});

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/traction/cmd_vel", rclcpp::SystemDefaultsQoS());
    state_pub_ = create_publisher<rosey_traction_msgs::msg::TractionState>(
      "/traction/state", rclcpp::SystemDefaultsQoS());

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(), "traction_node: %zu wheels, %.1f Hz, imu '%s', mass %.2f kg, %s",
      config.wheels.size(), rate_hz_, imu_topic_.c_str(), config.mass_kg,
      config.limiter.enabled ? "ENABLED" : "monitor mode");
  }

private:
  CoreConfig loadConfig()
  {
    CoreConfig c;

    bool enabled = declare_parameter("enabled", false);
    rate_hz_ = declare_parameter("rate_hz", 10.0);
    const auto joint_names = declare_parameter(
      "joint_names", std::vector<std::string>{
        "front_left_wheel_joint", "front_right_wheel_joint",
        "back_left_wheel_joint", "back_right_wheel_joint"});
    imu_topic_ = declare_parameter("imu_topic", std::string("/imu/data"));
    c.mass_kg = declare_parameter("mass_kg", 16.01);
    c.gravity = declare_parameter("gravity", 9.81);
    const auto kt = declare_parameter("kt_nm_per_a", std::vector<double>{1.77, 1.77, 1.77, 1.77});
    const auto i0 = declare_parameter("i0_a", std::vector<double>{0.15, 0.15, 0.15, 0.15});
    c.omega_min = declare_parameter("omega_min", 0.05);
    c.turn_bypass_threshold = declare_parameter("turn_bypass_threshold", 0.05);
    c.moving_cmd_min = declare_parameter("moving_cmd_min", 0.01);
    c.limiter.kp = declare_parameter("kp", 2.0);
    c.limiter.ki = declare_parameter("ki", 0.5);
    c.limiter.ki_bleed = declare_parameter("ki_bleed", 1.0);
    c.limiter.k_min = declare_parameter("k_min", 0.2);
    stale_timeout_ = declare_parameter("stale_timeout", 0.5);
    c.frozen_cycles = static_cast<int>(declare_parameter("frozen_cycles", int64_t{10}));
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
    imu_timeout_ = declare_parameter("imu_timeout", 0.5);
    pitch_axis_ = declare_parameter("pitch_axis", std::string("pitch"));
    pitch_sign_ = declare_parameter("pitch_sign", 1.0);

    Params & p = c.model;
    p.r = declare_parameter("wheel.r", p.r);
    p.b = declare_parameter("wheel.b", p.b);
    p.h = declare_parameter("wheel.h", p.h);
    p.lambda_s = declare_parameter("wheel.lambda_s", p.lambda_s);
    p.kc = declare_parameter("soil.kc", p.kc);
    p.kphi = declare_parameter("soil.kphi", p.kphi);
    p.n0 = declare_parameter("soil.n0", p.n0);
    p.n1 = declare_parameter("soil.n1", p.n1);
    p.c = declare_parameter("soil.c", p.c);
    p.phi = declare_parameter("soil.phi_deg", p.phi / kDegToRad) * kDegToRad;
    p.K = declare_parameter("soil.K", p.K);
    p.c1 = declare_parameter("soil.c1", p.c1);
    p.c2 = declare_parameter("soil.c2", p.c2);
    p.c3 = declare_parameter("soil.c3", p.c3);
    p.n_int = static_cast<int>(declare_parameter("n_int", static_cast<int64_t>(p.n_int)));

    // Phase 2 runs in monitor mode only. The closed loop is enabled in Phase 3.
    if (enabled) {
      RCLCPP_WARN(
        get_logger(), "enabled: true is not supported yet (Phase 2, monitor mode only); "
        "ignoring it, scale stays 1");
      enabled = false;
    }
    c.limiter.enabled = enabled;

    if (joint_names.size() != kt.size() || joint_names.size() != i0.size()) {
      throw std::invalid_argument(
              "joint_names, kt_nm_per_a and i0_a must have the same length");
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
      c.wheels.push_back(WheelConfig{joint_names[i], kt[i], i0[i]});
    }
    if (!(rate_hz_ > 0.0)) {throw std::invalid_argument("rate_hz must be > 0");}
    if (!(stale_timeout_ > 0.0)) {throw std::invalid_argument("stale_timeout must be > 0");}
    if (!(cmd_timeout_ > 1.0 / rate_hz_)) {
      throw std::invalid_argument("cmd_timeout must be longer than one timer period");
    }
    if (!(imu_timeout_ > 0.0)) {throw std::invalid_argument("imu_timeout must be > 0");}
    if (pitch_axis_ != "pitch" && pitch_axis_ != "roll") {
      throw std::invalid_argument("pitch_axis must be \"pitch\" or \"roll\"");
    }
    if (pitch_sign_ != 1.0 && pitch_sign_ != -1.0) {
      throw std::invalid_argument("pitch_sign must be 1.0 or -1.0");
    }
    const auto err = c.validate();
    if (!err.empty()) {throw std::invalid_argument("traction_node parameters: " + err);}

    wheel_inputs_.resize(c.wheels.size());
    for (size_t i = 0; i < c.wheels.size(); ++i) {
      wheel_index_[c.wheels[i].joint_name] = i;
    }
    return c;
  }

  void onJointStates(const control_msgs::msg::DynamicJointState & msg)
  {
    // Look wheels up by joint name, never by position in the message.
    for (auto & w : wheel_inputs_) {w.present = false;}
    const size_t n = std::min(msg.joint_names.size(), msg.interface_values.size());
    for (size_t j = 0; j < n; ++j) {
      const auto it = wheel_index_.find(msg.joint_names[j]);
      if (it == wheel_index_.end()) {continue;}
      const auto & iv = msg.interface_values[j];
      std::optional<double> velocity;
      std::optional<double> current;
      const size_t m = std::min(iv.interface_names.size(), iv.values.size());
      for (size_t k = 0; k < m; ++k) {
        if (iv.interface_names[k] == "velocity") {velocity = iv.values[k];}
        if (iv.interface_names[k] == "current") {current = iv.values[k];}
      }
      auto & w = wheel_inputs_[it->second];
      w.present = velocity.has_value() && current.has_value();
      w.omega = velocity.value_or(0.0);
      w.current = current.value_or(0.0);
    }
    last_joint_time_ = now();
    joints_new_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    // orientation_covariance[0] == -1 means the message carries no orientation.
    if (msg.orientation_covariance[0] == -1.0) {return;}
    double x = msg.orientation.x;
    double y = msg.orientation.y;
    double z = msg.orientation.z;
    double w = msg.orientation.w;
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(norm > 1e-6) || !std::isfinite(norm)) {return;}
    x /= norm; y /= norm; z /= norm; w /= norm;

    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    tilt_cos_ = 1.0 - 2.0 * (x * x + y * y);
    pitch_ = pitch_sign_ * (pitch_axis_ == "roll" ? roll : pitch);
    last_imu_time_ = now();
  }

  void onCmd(const geometry_msgs::msg::TwistStamped & msg)
  {
    last_cmd_ = msg;
    last_cmd_time_ = now();
  }

  static bool within(const std::optional<rclcpp::Time> & t, const rclcpp::Time & now, double s)
  {
    return t.has_value() && (now - *t).seconds() <= s;
  }

  void onTimer()
  {
    const auto t = now();
    const auto steady = std::chrono::steady_clock::now();
    const double dt = last_tick_ ?
      std::chrono::duration<double>(steady - *last_tick_).count() : 0.0;
    last_tick_ = steady;

    CycleInput in;
    in.wheels = wheel_inputs_;
    in.joints_fresh = within(last_joint_time_, t, stale_timeout_);
    in.joints_new = joints_new_;
    joints_new_ = false;
    in.imu_fresh = within(last_imu_time_, t, imu_timeout_);
    in.tilt_cos = tilt_cos_;
    in.pitch = pitch_;
    in.have_command = last_cmd_.has_value() &&
      within(last_cmd_time_, t, cmd_timeout_ - 1.0 / rate_hz_);
    in.cmd_linear_x = in.have_command ? last_cmd_->twist.linear.x : 0.0;
    in.cmd_angular_z = in.have_command ? last_cmd_->twist.angular.z : 0.0;
    in.dt = dt;

    auto out = core_->step(in);
    if (!last_cmd_.has_value()) {
      out.warnings.push_back("no command yet");
    } else if (!in.have_command) {
      out.warnings.push_back("command timed out, sending zero");
    }

    // Command: latest input with linear.x scaled, or zero if the input timed out.
    geometry_msgs::msg::TwistStamped cmd;
    if (in.have_command) {
      cmd = *last_cmd_;
      cmd.twist.linear.x *= out.scale;
    } else if (last_cmd_.has_value()) {
      cmd.header.frame_id = last_cmd_->header.frame_id;
    }
    cmd.header.stamp = t;
    cmd_pub_->publish(cmd);

    rosey_traction_msgs::msg::TractionState state;
    state.header.stamp = t;
    state.header.frame_id = "base_link";
    state.wheels.reserve(out.wheels.size());
    for (const auto & w : out.wheels) {
      rosey_traction_msgs::msg::WheelTraction m;
      m.joint_name = w.joint_name;
      m.omega = w.omega;
      m.current = w.current;
      m.torque = w.torque;
      m.load = w.load;
      m.slip_est = w.slip_est;
      m.theta_f = w.theta_f;
      m.s_max = w.s_max;
      m.t_max = w.t_max;
      m.estimate_valid = w.estimate_valid;
      m.limiting = w.limiting;
      state.wheels.push_back(m);
    }
    state.pitch = out.pitch;
    state.scale = out.scale;
    state.enabled = core_->config().limiter.enabled;
    state.bypass_turning = out.bypass_turning;
    state.status = out.status();
    state_pub_->publish(state);

    if (state.status != last_status_) {
      if (state.status == "ok") {
        RCLCPP_INFO(get_logger(), "status: ok");
      } else {
        RCLCPP_WARN(get_logger(), "status: %s", state.status.c_str());
      }
      last_status_ = state.status;
    }
  }

  std::unique_ptr<TractionCore> core_;

  double rate_hz_ = 10.0;
  double stale_timeout_ = 0.5;
  double cmd_timeout_ = 0.5;
  double imu_timeout_ = 0.5;
  std::string imu_topic_;
  std::string pitch_axis_;
  double pitch_sign_ = 1.0;

  std::unordered_map<std::string, size_t> wheel_index_;
  std::vector<WheelInput> wheel_inputs_;
  std::optional<rclcpp::Time> last_joint_time_;
  bool joints_new_ = false;

  double tilt_cos_ = 1.0;
  double pitch_ = 0.0;
  std::optional<rclcpp::Time> last_imu_time_;

  std::optional<geometry_msgs::msg::TwistStamped> last_cmd_;
  std::optional<rclcpp::Time> last_cmd_time_;

  std::optional<std::chrono::steady_clock::time_point> last_tick_;
  std::string last_status_;

  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<rosey_traction_msgs::msg::TractionState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rosey_traction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<rosey_traction::TractionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("traction_node"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
