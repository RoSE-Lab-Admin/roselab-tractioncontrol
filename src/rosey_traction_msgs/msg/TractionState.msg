# Traction controller state, published every control cycle on /traction/state.

std_msgs/Header header

WheelTraction[] wheels   # in the order of the joint_names parameter

float64 pitch            # pitch from the IMU [rad], 0 when the IMU is stale
float64 scale            # factor applied to linear.x this cycle, in [k_min, 1]
bool enabled             # false: monitor mode, scale is always 1
bool bypass_turning      # turning command passed through unchanged
string status            # "ok", or warnings separated by "; "
