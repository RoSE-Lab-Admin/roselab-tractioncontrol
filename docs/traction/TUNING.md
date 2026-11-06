# Traction control tuning

How `traction_node` turns motor current into a speed scale, which parameters change what, and how
to try changes offline before touching the rover. Parameters live in
`src/rosey_traction/config/traction.yaml`.

## What the controller does each cycle (10 Hz)

For each wheel:

1. Torque from current: `T = Kt * (|I| - I0) * sign(omega)`.
2. Load: `W = m * g * tilt_cos / 4`, with `tilt_cos = cos(roll) * cos(pitch)` from the IMU (1 if the IMU is stale).
3. Slip estimate `s` from `(W, T)` with the terramechanics model (Newton, bisection fallback).
4. Limits for this load: `s_max` from the slip table, `t_max = T(W, s_max)`.
5. Wheel error, in slip units: `e_i = max(s - s_max, (T - t_max) / t_max * s_max)`.

Then `e = max over wheels`, and the PI limiter gives the scale `k` applied to `linear.x`:

```
I  += e * dt               while e > 0
I  *= exp(-ki_bleed * dt)  while e <= 0
k   = clamp(1 - kp * max(e, 0) - ki * I, k_min, 1)
```

`k = 1` (no change) when: `enabled` is false (monitor mode), the turn bypass is active, or the
telemetry is stale or frozen. `angular.z` is never scaled.

## The torque term mainly backs up an invalid estimate

The torque term `(T - t_max) / t_max * s_max` is there for cycles where the slip estimate fails. The
estimator fails most often at very high slip, when the measured torque is above what the model gives
at s = 0.9. Without the torque term, the error would go silent exactly when slip is worst.

When the estimate is valid, the torque term rarely matters. The model torque rises with slip
(checked by the `TorqueAndSlipTermsAgreeInSign` test), so `T > t_max` and `s > s_max` always happen
together. It is also small. The torque curve flattens past `s_max`, so at W = 39.26 N:

| s     | T [N m] | dT/ds | torque term | slip term |
|-------|---------|-------|-------------|-----------|
| 0.363 | 3.413   | 2.0   | 0           | 0         |
| 0.45  | 3.546   | 1.13  | 0.014       | 0.087     |
| 0.60  | 3.654   | 0.43  | 0.026       | 0.237     |
| 0.90  | 3.723   | 0.14  | 0.033       | 0.537     |

So as a backup it is weak: with an invalid estimate the error is at most about 0.03, which gives
`k` around 0.93 from the P term, plus a slowly growing integral. If bench runs show the estimator
dropping out at high slip, options are to scale the torque term by the local slope `dT/ds` instead of
`t_max / s_max`, or to treat "invalid with T above t_max" as a fixed error such as `0.9 - s_max`.

## Parameters

| Parameter | Default | Effect | Notes |
|---|---|---|---|
| `enabled` | false | Monitor mode when false: estimates and `/traction/state` only, `k = 1`. | The only parameter that can change at runtime: `ros2 param set /traction_node enabled true`. Others need a restart. |
| `kp` | 2.0 | Immediate cut per unit slip error. `kp = 2` with `e = 0.1` cuts 20%. | Higher reacts faster to a slip spike but also to estimator noise. |
| `ki` | 0.5 | Cut that grows while slip stays above `s_max`. | Removes the steady error the P term leaves. Anti-windup caps `I` at `(1 - k_min) / ki`. |
| `ki_bleed` | 1.0 /s | How fast the integral decays once `e <= 0`. Sets recovery time: `I` falls to 37% in `1/ki_bleed` s, 5% in `3/ki_bleed` s. | Too high and `k` pumps back up while the wheel is still near its limit; too low and the rover stays slow after leaving the bad patch. |
| `k_min` | 0.2 | Lowest scale. The rover never drops below `k_min * linear.x`. | Keeps some forward motion; stopping completely on soft soil can dig in. |
| `turn_bypass_threshold` | 0.05 rad/s | Above this `|angular.z|`, commands pass through unchanged and the integral resets. | Skid steering slips by design, and the model is for straight driving. |
| `omega_min` | 0.05 rad/s | No estimate below this wheel speed. | Slip is not defined for a stopped wheel. |
| `kt_nm_per_a`, `i0_a` | 1.77, 0.15 | Current to wheel torque, per joint. | PLACEHOLDERS (bench mean). Errors here shift every estimate. At `s_max` (T = 3.41 N m), Kt 10% too low reads as s = 0.25; Kt 10% too high gives T = 3.75 N m, above T(0.9) = 3.72, so the estimate fails and only the torque term remains. Replace with the wheel characterization. |
| `mass_kg` | 16.01 | Wheel load `W`. | Update if payload changes. |
| `stale_timeout` | 0.5 s | `/dynamic_joint_states` older than this: `k = 1`, warning. | |
| `frozen_cycles` | 10 | Identical telemetry for this many messages while moving: `k = 1`, warning. | Moving = a wheel above `omega_min` or `|linear.x| > moving_cmd_min`. |
| `moving_cmd_min` | 0.01 m/s | See `frozen_cycles`. | |
| `cmd_timeout` | 0.5 s | Output goes to zero no later than this after the last `/cmd_vel`. | |
| `imu_timeout` | 0.5 s | IMU older than this: flat ground assumed. | |
| `pitch_axis`, `pitch_sign` | pitch, 1 | Reported pitch only. The load uses the full tilt and does not depend on these. | Settle in the bench tilt test. |
| `wheel.*`, `soil.*`, `n_int` | Octave values | The terramechanics model. | Refit `n0`, `n1`, `K` only from measured torque vs slip (Phase 5). |

Note: the model uses wheel radius 0.1515 m; `diff_drive` uses 0.150 m. They are kept separate on purpose.

## Estimator noise above s_max

Because `dT/ds` falls past `s_max`, the same current noise gives much larger slip noise there. With
1% current noise: about plus or minus 0.02 in `s` near `s_max`, about plus or minus 0.09 at `s = 0.6`.
`e` is the maximum over four wheels, so noise biases it upward. In practice this means:

- Below `s_max` the controller is quiet: in the replay, `k` stayed exactly 1 until `s` reached about 0.35.
- Above `s_max` the error is jumpy, and a high `kp` passes that straight to the speed.
- If real current noise is larger than 1%, consider filtering `e` (low-pass or median over a few
  cycles) before raising gains. Not implemented; Phase 5 decides based on recorded data.

## Trying gains offline

`replay_sim` runs the node's per-cycle logic (`TractionCore`) on a synthetic slip ramp: s = 0.1 for
2 s, up to 0.6 over 5 s, hold 1 s, back to 0.1 over 5 s, hold 5 s, at a constant 0.2 m/s command,
with seeded 1% current noise. Slip is prescribed, so the replay checks the controller logic, not how
the rover responds to being slowed.

```
colcon build --base-paths src/rosey_traction src/rosey_traction_msgs --packages-up-to rosey_traction
build/rosey_traction/replay_sim                         # table every 0.5 s
build/rosey_traction/replay_sim --kp=4 --ki_bleed=3     # override gains
build/rosey_traction/replay_sim --noise=0.02 --seed=7
build/rosey_traction/replay_sim --monitor               # k stays 1, LIMIT flags only
build/rosey_traction/replay_sim --csv > trace.csv       # every cycle
```

Results over 5 seeds (2026-11-06). Slip falls back below `s_max` at t = 10.4 s.

| Gains | First k < 1 at s | Mean k while s > 0.5 | k back to 0.95 at t |
|---|---|---|---|
| default (kp 2, ki 0.5, bleed 1) | 0.350 | 0.24 | 13.1 s |
| kp 1 | 0.350 | 0.38 | 13.1 s |
| kp 4 | 0.350 | 0.20 | 13.1 s |
| ki 2 | 0.350 | 0.20 | 13.3 s |
| bleed 0.3 | 0.350 | 0.24 | not within 18 s |
| bleed 3 | 0.350 | 0.24 | 11.4 s |
| default, 2% noise | 0.326 | 0.24 | 13.4 s |

At s = 0.6 the error is large enough that most gain sets sit at `k_min`; the gains matter most for
slip just above `s_max`, which is where the rover should spend its time once the loop is closed.
`ki_bleed` alone sets how quickly speed comes back.

`software_traction_replay` runs the same scenario as a test: `k` must be exactly 1 below
`s_max - 0.03`, drop only once slip reaches `s_max`, stay at or above `k_min`, only rise while
recovering, and be above 0.99 at the end.

## Not settled yet (needs the rover)

- Reversing: no estimate while a wheel turns backwards. Testing will decide how reversing is handled.
- Current sign convention and which `current` belongs to which wheel (bench checklist, Phase 4).
- Pitch axis and sign from the UM7 (tilt test).
- Kt and I0 per wheel (wheel characterization).
- Gains and any filtering (Phase 5, from recorded runs).
