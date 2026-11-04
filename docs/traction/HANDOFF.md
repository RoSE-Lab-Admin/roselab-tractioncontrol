# RoSEy Traction Control: Claude Code Handoff

Owner: Ayooluwa (Ayo) Adeyinka, RoSE Lab, Colorado School of Mines
Repo: `RoSE-Lab-Admin/CubeRover`, base branch `dev-fitRoseyRefactored` (last checked at `4475bd0`, Sep 30 2026)
Working branch: `feature/traction-control` (create it off `dev-fitRoseyRefactored`)
Semester deadline: Dec 5, 2026

---

## 0. Rules for Claude Code (read first)

1. **Read this whole file and `PROGRESS.md` before doing anything.** Update `PROGRESS.md` at the end of every session: what you did, what's next, open questions.
2. **Work one phase at a time.** Every phase ends with a **STOP** checkpoint. At a STOP, summarize what changed, list anything you could not verify, and wait for Ayo.
3. **The code wins.** If anything here disagrees with the repo, trust the repo, tell Ayo, and note it in `PROGRESS.md`.
4. **Never do these without Ayo saying yes in chat for that specific action:**
   - `git push`, opening PRs, rebasing or force-anything.
   - Anything that talks to hardware: serial devices (`/dev/tty*`), launching on the rover, `ros2 topic pub` to a live rover, SSH to the Pi.
   - Editing the firmware (`Arduino_ROS/`), the hardware interface (`src/rosey_drivers/`), or the git submodules (`src/um7/`).
   - Deleting files that you did not create.
5. **Allowed without asking:** reading anything, creating and editing files inside the new packages and `docs/traction/`, the one launch-file change in Phase 2, building, running unit tests, running with `use_mock_hardware:=true`, local commits on `feature/traction-control` with clear messages.
6. **Style:** C++17, follow the existing code style in `src/rosey_drivers`. In docs and comments, write plainly and do not use em dashes or en dashes.
7. **When unsure about physics or a design choice, ask.** Ayo knows the model; don't invent parameters.

---

## 1. What we are building (short)

Terramechanics-based **straight-line traction control** for the RoSEy 4-wheel skid-steer rover.

- **Model:** Ding et al. 2009 (IROS), Eqs. 16 to 22: Bekker pressure-sinkage with Wong-Reece stress distribution, Janosi-Hanamoto shear, shearing radius r_s = r + 0.5h, slip-dependent sinkage exponent N = n0 + n1·s. Validated against Ding's measured data (drawbar pull RMSE 3.1 N) after adding the lug-entry angle θf′ (Ding Eq. 17) to the shear displacement.
- **Offline:** lookup table of s_max (by wheel load) and s_min (by load and incline). Already generated: `reference/slip_table.h`.
- **Onboard:** estimate each wheel's slip from its load W and torque T by inverting the model (Newton-Raphson on θf and s). Torque comes from motor current.
- **Control:** if any wheel's slip goes above s_max (or torque above T(s_max)), scale down the commanded forward speed with a PI loop. Turning is out of scope: pass turning commands straight through.

The Octave reference implementation is `reference/rosey_terramechanics_octave.m`. **The C++ port must match it** (see the golden CSVs).

---

## 2. Decisions already made

| Decision | Choice |
|---|---|
| Where it runs | Raspberry Pi, as a ROS 2 `rclcpp` node. Not the Teensy. |
| Target branch | `dev-fitRoseyRefactored` |
| Code location | New packages under `src/`. Minimal edits to existing packages. |
| Integration point (this semester) | A node between the operator and diff_drive that limits `linear.x`. |
| Per-wheel commands | Next semester (needs hardware interface changes). Not now. |
| First mode | **Monitor mode** (estimate and publish, pass commands through). Closed loop only after validation. |
| Motor torque constants | Config values. Wheel characterization data is coming; use placeholders until then. |
| Core math | Plain C++17 with no ROS dependency, so it is unit-testable and portable. |

---

## 3. Repo facts (refactor branch) to rely on and verify

Verify each row in Phase 0. Report any mismatch.

| Fact | Where |
|---|---|
| Packages: `rosey_bringup` (launch, config), `rosey_description` (URDF, meshes), `rosey_drivers` (ros2_control hardware plugin), `um7` (git submodules: `um7`, `serial-ros2`) | `src/` |
| Joint order is **FL, FR, BL, BR**. RoboClaw 1 = front (M1 FL, M2 FR), RoboClaw 2 = back (M1 BL, M2 BR). | `rosey_description/.../roseybot_control.urdf.xacro`, `Arduino_ROS/motor_driver.cpp` |
| `read()` maps telemetry to joints **by index**; per-wheel `current` is in amps (raw is 10 mA steps, divided by 100) | `rosey_drivers/hardware/roseybot_system.cpp`, `wheel.hpp` |
| Each joint exports state interfaces `position` [rad], `velocity` [rad/s], `current` [A], `voltage` [V], `pwm` [%] | URDF + `roseybot_system.cpp` |
| `joint_state_broadcaster` publishes all of them on `/dynamic_joint_states` (`control_msgs/msg/DynamicJointState`) | standard behavior; confirm with mock hardware |
| `write()` sends per-side speeds only (`'m' left right`) | `roseybot_system.cpp` |
| Firmware also has per-wheel `'d' <index> <speed>`, index 1 FL, 2 FR, 3 BL, 4 BR. **The comment in `commands.h` says FL,BL,FR,BR and is stale.** | `Arduino_ROS/motor_driver.cpp` |
| Controller manager runs at **10 Hz**; diff_drive `linear.x.max_acceleration: 0.5`, `cmd_vel_timeout: 0.5`, `wheel_radius: 0.150`, `wheel_separation: 0.3365` | `rosey_bringup/bringup/config/roseybot_controllers.yaml` |
| diff_drive input is remapped to `/cmd_vel` in the spawner args | `rosey_bringup/bringup/launch/ros2_control.launch.py` |
| `use_mock_hardware` launch arg swaps in `mock_components/GenericSystem` | same launch file + URDF |
| IMU is a UM7 via `umx_driver`, frame `imu_link`, `tf_ned_to_enu: true`, mounted with rpy 0 0 0 relative to `base_link`. **Topic name not yet confirmed.** | `rosey_bringup/bringup/config/um7.yaml`, `imu_boot.launch.py`, URDF |
| No CI, no unit tests, no devcontainer on this branch (they exist on `main`). ROS distro assumed Jazzy (main uses Jazzy). | `.github/` absent |

---

## 4. Target architecture

```
/cmd_vel (TwistStamped) ──▶ traction_node ──▶ /traction/cmd_vel ──▶ roseybot_base_controller ──▶ hardware
                                ▲
     /dynamic_joint_states ─────┤  per-wheel velocity [rad/s] and current [A]
     IMU topic (param) ─────────┘  orientation → pitch
                                │
                                └──▶ /traction/state (rosey_traction_msgs/TractionState)
```

### New packages

```
src/rosey_traction_msgs/          ament_cmake, messages only
  msg/WheelTraction.msg           joint_name, omega, current, torque, load, slip_est, theta_f,
                                  s_max, t_max, estimate_valid (bool), limiting (bool)
  msg/TractionState.msg           std_msgs/Header header, WheelTraction[] wheels, float64 pitch,
                                  float64 scale, bool enabled, bool bypass_turning, string status

src/rosey_traction/               ament_cmake
  include/rosey_traction/
    terramechanics.hpp            Params struct, normalStress, shearStress, wheelForces,
                                  simpson, solveEntryAngle
    slip_estimator.hpp            Newton-Raphson (θf, s) from (W, T); bisection fallback
    slip_table.h                  copied from docs/traction/reference (generated, do not hand-edit)
    slip_table_lookup.hpp         sMax(W) and sMin(W, alpha) with interpolation, -1 = infeasible
    current_to_torque.hpp         T = Kt·(|I| − I0)·sign(ω), clamp at 0 below I0
    traction_limiter.hpp          PI on slip error, anti-windup, scale in [k_min, 1]
  src/                            .cpp for the above (no ROS includes) + traction_node.cpp
  config/traction.yaml
  launch/traction.launch.py
  test/                           gtests named software_* (see Phase 1)
```

### Node behavior (`traction_node`, timer at 10 Hz)

Per wheel i (look wheels up **by joint name**, never by index):
1. Read ω_i and I_i from the latest `/dynamic_joint_states`.
2. T_i = Kt_i·(|I_i| − I0_i), sign from ω_i. If |ω_i| < `omega_min` (default 0.05 rad/s): no estimate this cycle.
3. W_i = m·g·cos(pitch)/4 (equal split for now).
4. (θf_i, s_i) = estimate(W_i, T_i), warm-started from last cycle. Mark invalid if it fails.
5. s_max_i = sMax(W_i); T_max_i = T at (W_i, s_max_i).

Control:
- If |angular.z| of the incoming command > `turn_bypass_threshold`: pass through unchanged, reset the integrator, `bypass_turning = true`.
- e = max over valid wheels of (s_i − s_max_i); also e > 0 if any T_i > T_max_i (use the torque-based error, scaled to slip units, documented in code).
- PI: integral accumulates only while e > 0, bleeds toward 0 at `ki_bleed` when e ≤ 0. scale k = clamp(1 − Kp·e − Ki·∫e, k_min, 1).
- If `enabled == false` (default): **k = 1 always** (monitor mode), still publish state.
- Publish `/traction/cmd_vel` = incoming command with `linear.x *= k`, **on every timer tick** using the latest incoming command (diff_drive times out after 0.5 s). If no command for > 0.5 s, publish zero velocity.

Guards:
- `/dynamic_joint_states` older than `stale_timeout` (0.5 s), or all 4 currents and velocities identical for `frozen_cycles` cycles: invalid, k = 1, status warning.
- IMU older than 0.5 s: pitch = 0, status warning.

### `config/traction.yaml` (starting values)

```yaml
traction_node:
  ros__parameters:
    enabled: false                 # monitor mode until validated
    rate_hz: 10.0
    joint_names: [front_left_wheel_joint, front_right_wheel_joint, back_left_wheel_joint, back_right_wheel_joint]
    imu_topic: "/imu/data"         # PLACEHOLDER: confirm in Phase 0
    mass_kg: 16.01                 # PLACEHOLDER: confirm with Ayo
    # Motor model, per joint, same order as joint_names. PLACEHOLDERS: mean of bench fit
    # (Motor 1..4 = 1.61, 1.85, 1.88, 1.74 N*m/A). Wheel characterization will replace these.
    kt_nm_per_a: [1.77, 1.77, 1.77, 1.77]
    i0_a: [0.15, 0.15, 0.15, 0.15]
    omega_min: 0.05
    turn_bypass_threshold: 0.05    # rad/s
    kp: 2.0                        # PLACEHOLDER gains, tune in Phase 5
    ki: 0.5
    ki_bleed: 1.0
    k_min: 0.2
    stale_timeout: 0.5
    frozen_cycles: 10
    # Terramechanics (RoSEy wheel + Ding soil), same as the Octave script
    wheel: {r: 0.1515, b: 0.05158, h: 0.013, lambda_s: 0.5}
    soil:  {kc: 15.6, kphi: 2407.4, n0: 0.81, n1: 1.16, c: 250.0, phi_deg: 31.9, K: 0.011,
            c1: 0.5, c2: -0.3, c3: 0.0}
    n_int: 60
```

Note: the model uses r = 0.1515 m (Hopkins). diff_drive uses `wheel_radius: 0.150`. Keep them separate; flag the difference to Ayo, don't "fix" either.

---

## 5. Phases

### Phase 0: Orient and check the environment (read only)

1. `git status`, confirm branch `feature/traction-control` based on `dev-fitRoseyRefactored`. `git submodule update --init --recursive` if `src/um7/*` is empty (ask first if it needs network credentials).
2. Verify every row in Section 3. Produce a short table: fact, confirmed / different, file:line.
3. Find the **UM7 IMU topic name and message type** from the `umx_driver` source in `src/um7/um7`. Report it.
4. Check the toolchain: `ros2 --version` or `printenv ROS_DISTRO`, `colcon --version`, compiler, GTest. If ROS 2 is not installed on this laptop, **stop and tell Ayo**. Options to offer: (a) copy `.devcontainer/` from `origin/main` (the lab's existing Jazzy dev container), (b) build on the Pi. Do not install ROS yourself.
5. Create `docs/traction/PROGRESS.md` from the template at the bottom of this file.

**STOP.** Report findings and environment status.

### Phase 1: Core library + unit tests (no ROS needed in the code)

Port from `reference/rosey_terramechanics_octave.m` (functions section) and `reference/make_golden.m`:

1. `terramechanics.hpp/.cpp`: exact port, including the θf′ term (`Rj = r + h`), Simpson with `n_int` even intervals, bisection for θf on [1e-4, 80°] that returns "no solution" (not NaN propagation) when W(80°) < target.
2. `slip_estimator.hpp/.cpp`: port `estimateSlip` exactly (finite-difference Jacobian with steps [1e-5 rad, 1e-4], Cramer's rule, step limits [5°, 0.1], clamps θf ∈ [1e-4, 80°] and s ∈ [0, 0.95], tolerances 0.1%, singular-Jacobian guard, 20 iterations). Add a fallback: if Newton fails, bisection on s ∈ [0, 0.9] using T(s) at the given W (see `slipFromTorque` in `reference/rosey_current_tradestudy.m`). Return a struct {s, theta_f, valid, iterations, used_fallback}.
3. `slip_table_lookup`: linear interpolation of `S_MAX` over load (clamp outside grid); bilinear `S_MIN` over load and incline; return "infeasible" if any corner is -1.
4. `current_to_torque` and `traction_limiter` as in Section 4.
5. Tests in `src/rosey_traction/test/`, registered so the ctest names start with `software_` (match the `main` branch convention: `ament_add_gtest(software_<name> ...)`). Copy the golden CSVs into `test/data/` and pass the path via a compile definition.
   - `software_terramechanics`: every row of `golden_forces.csv` (W, DP, T, FT, R) within **0.5%** (or 1e-3 absolute when |value| < 0.2). Every row of `golden_curves.csv` (θf within 1e-4 rad, DP and T within 0.5%).
   - `software_slip_estimator`: every row of `golden_estimator.csv` recovers s within **0.005** and θf within 0.2°. Also with ±1% noise on W and T: |s error| < 0.02 for s ≤ 0.3. Also: invalid input (T < 0, W beyond range) returns `valid = false` without throwing.
   - `software_slip_table`: grid points return table values exactly; midpoints interpolate; infeasible cells flagged.
   - `software_traction_limiter`: k = 1 when e ≤ 0; k decreases with sustained e > 0; never below k_min; integrator bleeds off; monitor mode always 1.
6. Build: `colcon build --packages-select rosey_traction` and `colcon test --packages-select rosey_traction`, then `colcon test-result --verbose`.
7. Optional: time 4 estimator calls; report ms per cycle (target < 5 ms on the laptop).

**STOP.** Report test results and timing. Commit locally.

### Phase 2: Messages + node in monitor mode

1. `rosey_traction_msgs` package with the two messages in Section 4.
2. `traction_node.cpp`: parameters from `config/traction.yaml`, subscriptions (`/dynamic_joint_states`, IMU topic param, `/cmd_vel` as `geometry_msgs/msg/TwistStamped`), timer, publishers (`/traction/cmd_vel`, `/traction/state`). Pitch from the IMU quaternion in the `imu_link` frame (mounted aligned with `base_link`); document the axis convention you used and why.
3. `launch/traction.launch.py`: starts the node with the yaml.
4. **One edit to an existing file**, `rosey_bringup/bringup/launch/ros2_control.launch.py`: add launch arg `traction` (default `false`). When true, the diff_drive spawner remaps its input to `/traction/cmd_vel` instead of `/cmd_vel`, and the launch includes `rosey_traction/launch/traction.launch.py`. Default behavior must be byte-for-byte the same as before for the robot. Add `rosey_traction` as an `exec_depend` of `rosey_bringup` only if needed for the include.
5. Mock test: `ros2 launch rosey_bringup ros2_control.launch.py use_mock_hardware:=true traction:=true`. Confirm:
   - `/dynamic_joint_states` contains the `current` interface (mock may report 0; the node must handle that as "no estimate", not crash).
   - `/traction/state` at ~10 Hz.
   - A `TwistStamped` on `/cmd_vel` comes out on `/traction/cmd_vel` unchanged (monitor mode), and diff_drive responds (wheel velocities in `/joint_states`).
   - Stopping the `/cmd_vel` publisher makes `/traction/cmd_vel` go to zero within 0.5 s.
6. A `software_` gtest or launch test for the node is a plus, not required.

**STOP.** Show the launch-file diff and mock-test output. Commit locally.

### Phase 3: Closed-loop logic, still on mock

1. Make the limiter path live when `enabled: true`.
2. Add a small offline test harness (`scripts/replay_sim.py` or a gtest) that feeds synthetic (ω, I, pitch) sequences into the node logic: a slip ramp from 0.1 to 0.6 at constant command. Show that k drops when s passes s_max and recovers when slip falls. Plot or print the trace.
3. Document tuning knobs in `docs/traction/TUNING.md`.

**STOP.** Show the trace. Commit locally.

### Phase 4: Hardware test support (Claude writes, Ayo runs)

Write, don't run:
1. `docs/traction/BENCH_CHECKLIST.md`:
   - Lifted-wheel test: command one wheel at a time with `'d'` (index 1 FL, 2 FR, 3 BL, 4 BR), confirm which `current` changes and its sign convention.
   - Tilt test: confirm pitch sign and axis from the UM7 (nose up = ?).
   - Telemetry timing: measure `read()` round trip (log timestamps of `/dynamic_joint_states`) to see whether 20 Hz is possible.
   - Fill in Kt/I0 per wheel once the wheel characterization data arrives.
2. `scripts/record_run.sh`: `ros2 bag record` of `/dynamic_joint_states`, IMU topic, `/cmd_vel`, `/traction/cmd_vel`, `/traction/state`, `/tf`, plus an OptiTrack topic placeholder.
3. `scripts/analyze_run.py`: read a bag, compute kinematic slip from OptiTrack ground speed (s = 1 − v/(r_s·ω)), compare to `slip_est` per wheel, report RMSE for s ≤ 0.35, and plot both.

**STOP.** Ayo runs the bench checklist and sandbox tests.

### Phase 5: Tune with real data (after Ayo's runs)

Use Ayo's bags to set Kt/I0, gains, filtering. Possibly refit n0, n1, K if the measured torque vs slip disagrees with Ding's soil. Only with Ayo.

### Later (not this semester)

Per-wheel control: chainable ros2_control controller between diff_drive and the hardware, plus `write()` sending four `'d'` commands. Do not start this without Ayo.

---

## 6. Open items

| Item | Status |
|---|---|
| UM7 topic name and pitch axis | Phase 0 (name), bench (axis/sign) |
| Kt and I0 per wheel | Waiting on wheel characterization; placeholders in yaml |
| Rover mass | 16.01 kg placeholder (Hopkins), confirm |
| ROS 2 setup on Ayo's laptop | Phase 0 |
| OptiTrack topic name | Ask Ayo before Phase 4 analysis |
| Model wheel radius 0.1515 vs diff_drive 0.150 | Flag only |

---

## 7. Reference files (`docs/traction/reference/`)

| File | What it is |
|---|---|
| `rosey_terramechanics_octave.m` | Main Octave model: forces, θf bisection, Newton estimator, lookup table export. **Source of truth for the port.** |
| `make_golden.m` | Generates the golden CSVs below with the exact same model |
| `golden_forces.csv` | W, DP, T, FT, R at fixed (θf, s) for RoSEy and Ding Wh2 |
| `golden_curves.csv` | θf, sinkage z, DP, T at matched loads, s = 0 to 0.6 |
| `golden_estimator.csv` | (W, T) inputs with the true s and θf the estimator must recover |
| `slip_table.h` | Generated s_max / s_min lookup table (RoSEy, Ding soil) |
| `rosey_validation.m` | Model vs Ding 2009 measured data (why the θf′ term exists) |
| `rosey_current_tradestudy.m` | Current to torque fit, per-side vs per-wheel study, `slipFromTorque` |

---

## PROGRESS.md template

```markdown
# Traction control progress log

## Current phase
Phase N: <name>. Status: in progress / waiting at STOP.

## Session log
### YYYY-MM-DD
- Did:
- Verified:
- Could not verify:
- Next:

## Open questions for Ayo
-
```
