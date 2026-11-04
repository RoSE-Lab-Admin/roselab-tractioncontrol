# Traction control progress log

## Current phase
Phase 1: Core library + unit tests. Status: waiting at STOP. Committed locally.

## Session log
### 2026-11-04 (first attempt)
- Claude Code was started in the handoff folder, not the repo. No clone was found then. Superseded below.

### 2026-11-04
- Did:
  - Read HANDOFF.md and `reference/`. Verified the Section 3 repo facts in `~/Documents/CubeRover`.
  - Repo: branch `feature/traction-control`, HEAD `c68d5a1` (docs commit) on top of
    `origin/dev-fitRoseyRefactored` at `4475bd0`. Submodules `src/um7/um7` (1.0.1) and
    `src/um7/serial-ros2` are checked out.
  - Working tree: `.gitignore` modified (adds `.claude/settings.local.json`, also already in
    `.git/info/exclude`), untracked `.DS_Store` and `.claude/`. Left alone; these are Ayo's.
- Verified (file:line):
  - Packages in `src/`: rosey_bringup, rosey_description, rosey_drivers, um7 (submodules um7, serial-ros2).
  - Joint order FL, FR, BL, BR: `rosey_description/description/ros2_control/roseybot_control.urdf.xacro:42-45`.
    Firmware indices 1 FL, 2 FR, 3 BL, 4 BR: `Arduino_ROS/motor_driver.cpp:44-47`.
    Stale comment (FL,BL,FR,BR): `Arduino_ROS/commands.h:11`.
  - `read()` maps by index, current = raw / 100 A: `rosey_drivers/hardware/roseybot_system.cpp:271-281`,
    `wheel.hpp:40-44`. On serial timeout, `read()` logs and keeps the old values (`roseybot_system.cpp:282-284`).
  - Five state interfaces: URDF xacro `:9-13`, exported in `roseybot_system.cpp:139-153`.
  - `write()` sends `'m' left right` using joints 0 and 1: `roseybot_system.cpp:297`, firmware
    `motor_driver.cpp:53-54` (both RoboClaws get SpeedM1M2(l, r)).
  - Controllers yaml (`rosey_bringup/bringup/config/roseybot_controllers.yaml`): update_rate 10 (:3),
    wheel_separation 0.3365 (:15), wheel_radius 0.150 (:17), cmd_vel_timeout 0.5 (:33),
    linear.x.max_velocity 0.3 (:42), linear.x.max_acceleration 0.5 (:44).
  - diff_drive remap to `/cmd_vel`: `ros2_control.launch.py:69`. `use_mock_hardware`: launch `:19-27, :74-77`, xacro `:21-23`.
  - Teleop publishes TwistStamped (`joystick.yaml:28`, `publish_stamped_twist: true`) on default `/cmd_vel`.
  - UM7: `um7.yaml` frame_id imu_link (:4), tf_ned_to_enu true (:9). IMU joint rpy 0 0 0, xyz
    (0.16256, -0.05, 0.017088) from base_link: `roseybot_description.urdf.xacro:77-89`.
  - IMU topic: `/imu/data`, `sensor_msgs/msg/Imu` (`src/um7/um7/src/um7_driver.cpp:386`, node `um7_driver`,
    no namespace in `imu_boot.launch.py`). Also `/imu/rpy` (Vector3Stamped, :388).
  - No `.github/`, `.devcontainer/` or tests on this branch. `origin/main` has them; its Dockerfile is
    `FROM osrf/ros:jazzy-desktop`, CI runs `colcon test --ctest-args -R "software_"`.
  - Toolchain: see "Environment" below (Ayo pointed to a source build of Jazzy on the Mac).
- Could not verify:
  - `/dynamic_joint_states` content with mock hardware (ros2_control not installed, see below).
- Next:
  - Ayo decides how to get ros2_control (needed from Phase 2), answers the open questions, then Phase 1.

### 2026-11-04 (Phase 1)
- Did:
  - ros2_control overlay at `~/ros2_control_ws` (option a). Cloned (jazzy branches where they exist):
    ros2_control, ros2_controllers, control_msgs, realtime_tools, control_toolbox, ros2_control_cmake
    (jazzy); generate_parameter_library, RSL, cpp_polyfills (main); backward_ros (foxy-devel);
    xacro (ros2); angles (ros2); diagnostics (ros2); pal_statistics (humble-devel).
    Installed: Homebrew `tl-expected`; pip in `~/ros2_venv`: jinja2, typeguard, filelock, coverage.
    First build: 12 packages OK, then pal_statistics failed (boost headers not on the include path).
    Built pal_statistics alone with `-DCMAKE_CXX_FLAGS=-I/opt/homebrew/include`: OK.
    Resuming the rest was blocked by Claude Code's permission check (building code cloned from GitHub).
    Ayo needs to run the resume command (below) or allow it.
  - Created `src/rosey_traction` (ament_cmake, no ROS deps yet): terramechanics, slip_estimator,
    slip_table_lookup (+ copied slip_table.h), current_to_torque, traction_limiter, 4 gtests
    `software_*`, golden CSVs in `test/data/`, `bench_estimator` timing tool. Defaults to Release build.
- Verified:
  - `colcon build --packages-up-to rosey_traction`: clean (only CMake deprecation warnings).
  - 28 test cases, 26 pass. All golden_forces (40 rows), golden_curves (52 rows) and
    golden_estimator (15 rows, cold and warm start) pass. Slip table, limiter and current-to-torque pass.
  - Fails: `SlipEstimator.OnePercentNoise` at s_true = 0.3 only, when W and T noise have opposite
    signs: |s error| 0.021 to 0.027 vs the 0.02 spec. Estimator converges; the error is the model's
    own sensitivity (dT/ds falls from ~9 to ~3 N m per unit slip between s = 0.1 and 0.3 at 39.3 N).
    Worst corner / Gaussian(1%) rms at 39.3 N: s=0.2 0.011/0.008, s=0.3 0.024/0.016, s=0.35 0.034/0.023.
  - Timing (Release, M-series Mac), 4 wheels: warm start 0.017 ms, cold 0.18 ms, all-fallback 5.4 ms.
- Design choices made (flag for Ayo):
  - Integrator bleed is exponential, I *= exp(-ki_bleed*dt), ki_bleed in 1/s.
  - P term uses max(e, 0) so recovery after slip drops is set by the bleed.
  - Anti-windup: I clamped to [0, (1 - k_min)/ki].
  - sMin: incline above 20 deg returns infeasible; below 0 clamps to 0; load clamps to grid.
    Zero-weight corners ignored, so grid points return their own value.
  - Fallback is valid only if T_m is between T(W, 0) and T(W, 0.9).
  - currentToTorque returns signed torque; the estimator rejects T < 0. Reverse driving needs a
    decision in Phase 2 (use |T|, or no estimate).
- Resolved: noise test switched to seeded Gaussian 1% (Box-Muller on mt19937, portable across
  standard libraries), 500 samples per case. RMS error: s=0.05 0.002, 0.1 0.003, 0.2 0.008,
  0.3 0.014 to 0.018. All 28 tests pass.
- Next: Ayo runs the overlay resume command, then Phase 2.

Overlay resume command (run in a terminal):
```
source ~/ros2_venv/bin/activate && source ~/ros2_jazzy/install/setup.zsh
cd ~/ros2_control_ws && colcon build --packages-up-to controller_manager joint_state_broadcaster \
  diff_drive_controller control_msgs xacro --packages-skip pal_statistics pal_statistics_msgs \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=$VIRTUAL_ENV/bin/python3
```
Then each shell: source `~/ros2_control_ws/install/setup.zsh` after the Jazzy setup.

## Environment (decided 2026-11-04)
- ROS 2 Jazzy built from source at `~/ros2_jazzy`, Python venv `~/ros2_venv` (Python 3.11).
  Ayo said `~/ros_env`, which does not exist; `~/ros2_venv` is the one `~/.zshrc` uses.
- Setup, every shell: `source ~/ros2_venv/bin/activate && source ~/ros2_jazzy/install/setup.zsh`
  (source in the current shell, not through a pipe, or the env is lost).
- Build only with `colcon build --packages-up-to <pkg>`. rosey_drivers and the UM7 submodules may not
  build on macOS.
- Verified: `ros2` CLI, colcon, AppleClang 17, CMake 4.4.3. GTest works: a throwaway
  `ament_add_gtest(software_smoke ...)` package built and passed under colcon (scratchpad, not the repo).
- Present: rclcpp, rclcpp_lifecycle, sensor_msgs, geometry_msgs, std_msgs, nav_msgs, tf2, tf2_ros,
  tf2_geometry_msgs, rosidl_default_generators, ament_cmake_gtest, gtest_vendor, gmock_vendor,
  ament_cmake_pytest, launch_ros, launch_testing(_ros), rosbag2, robot_state_publisher, pluginlib.
- Missing (ros2_control stack): controller_manager, hardware_interface (includes mock_components),
  controller_interface, ros2_control_test_assets, ros2controlcli, joint_state_broadcaster,
  diff_drive_controller (ros2_controllers), control_msgs, control_toolbox, realtime_tools,
  transmission_interface, joint_limits, generate_parameter_library, parameter_traits, rsl, tcb_span,
  tl_expected, backward_ros.
- Also missing: xacro, joy, teleop_twist_joy, robot_localization, umx_driver/serial (repo submodules,
  not built).

## Answered
- 2026-11-04: Rover mass 16.01 kg confirmed. ros2_control: build from source into an overlay on the Mac (option a).
- 2026-11-04 (corrected, from the project manager): Kt is the wheel torque/current constant, not axle.
  T = Kt*(|I| - I0) is wheel torque directly, no gear ratio. This is what the code assumes.
- 2026-11-04: Noise test uses the Octave demo's noise model (1%*randn on W and T, seeded), RMS slip
  error < 0.02 for s <= 0.3 (option b). Worst-case corners are no longer tested.

## Open questions for Ayo
- ros2_control is missing. Phase 1 does not need it. Phase 2 needs `control_msgs` to compile the node
  (`DynamicJointState`), and the mock test needs controller_manager, mock_components,
  joint_state_broadcaster, diff_drive_controller and xacro. Options: build those from source into an
  overlay workspace on the Mac, or run the Phase 2 mock test on the Pi.
- `colcon build --packages-up-to rosey_bringup` pulls in rosey_drivers and umx_driver (exec_depends in
  `rosey_bringup/package.xml:21,25`), which may not build on macOS. The Phase 2 launch-file check may need
  the Pi or a different build command.
- UM7 ENU quaternion: in ENU mode the driver remaps the quaternion as w=q2, x=q1, y=-q3, z=q0
  (`um7_driver.cpp:183-186`) and the rpy topic swaps roll and pitch (`:282-286`). Taking pitch from the
  quaternion the obvious way may give roll or the wrong sign. Plan: make the pitch axis and sign
  parameters, and settle them with the bench tilt test.
- Stale telemetry: on a serial timeout `read()` keeps old values, and `/dynamic_joint_states` still gets a
  fresh stamp. The stale_timeout guard will not catch this; only the frozen-values guard will.
- The IMU is started by `hardware_startup.launch.py`, not `ros2_control.launch.py`. If traction is
  included from `ros2_control.launch.py`, the IMU only runs when launched through `hardware_startup`.
- slip_table.h header names `rosey_terramechanics.m`; the reference script is `rosey_terramechanics_octave.m`.
- Mock hardware: `current` may read 0 or NaN in GenericSystem. The node will treat both as "no estimate".
