# Traction estimate for one wheel, one control cycle.

string joint_name

float64 omega         # wheel speed from /dynamic_joint_states [rad/s]
float64 current       # motor current from /dynamic_joint_states [A]
float64 torque        # Kt * (|current| - I0) * sign(omega) [N m]
float64 load          # normal load used by the estimator [N]

float64 slip_est      # estimated slip [-], NaN when estimate_valid is false
float64 theta_f       # estimated entry angle [rad], NaN when estimate_valid is false
float64 s_max         # slip at peak drawbar pull for this load (slip table) [-]
float64 t_max         # model torque at (load, s_max) [N m], NaN if the model has no solution

bool estimate_valid   # false: wheel too slow, reversing, bad telemetry, or estimator failed
bool limiting         # this wheel is above s_max or t_max this cycle
