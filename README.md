# AUV Fault-Tolerant Control — ROS 2 Humble + Gazebo

A ROS 2 Humble / Gazebo Classic 11 implementation of the fault-tolerant
control scheme described in

> Zhang, Z.; Wu, Y.; Zhou, Y.; Hu, D.
> *Fault-Tolerant Control of Autonomous Underwater Vehicle Actuators
> Based on Takagi and Sugeno Fuzzy and Pseudo-Inverse Quadratic Programming
> under Constraints.* **Sensors 2024, 24, 3029.**
> DOI: 10.3390/s24103029

The stack reproduces the three algorithmic blocks the paper introduces:

1. a **T-S (Takagi-Sugeno) fuzzy state-feedback controller** with 6 rules on
   the 2D premise grid `θ₁ ∈ {0.5, 1.0} m/s`, `θ₂ ∈ {-0.1, 0, +0.1} rad/s`
   (Eqs. 18-25);
2. a **weighted pseudo-inverse thrust allocator** whose priority matrix `W`
   is driven by fault factors `f_i` via `w_i = exp(1/f_i − 1)` (Eqs. 31-37);
3. an **active-set QP re-allocator** for the constrained case
   `u_min ≤ u ≤ u_max` (Eqs. 39-49).

All three are wrapped in a standard ROS 2 node that drives a Gazebo
simulation of a torpedo-shaped AUV with 4 redundant actuator channels
(the Figure 3 layout).

---

## Workspace layout

```
auv_ftc_ws/
└── src/
    ├── auv_description/   # xacro URDF, RViz config
    ├── auv_gazebo/        # underwater world + spawn launch
    ├── auv_control/       # T-S fuzzy + pseudo-inv + QP + ROS 2 node
    └── auv_bringup/       # top-level "run everything" launch
```

---

## Prerequisites

| Component          | Version  |
|--------------------|----------|
| Ubuntu             | 22.04    |
| ROS 2              | Humble   |
| Gazebo             | Classic 11 |
| Eigen3             | ≥ 3.3    |

Install system dependencies once:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-xacro ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbridge-server \
    python3-matplotlib python3-numpy \
    libeigen3-dev
```

---

## Build

```bash
cd ~/auv_ftc_ws        # root of the workspace you unzipped
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The first build compiles the `InjectFault.srv` service interface and the
three executables:

| Executable                   | What it does                              |
|------------------------------|-------------------------------------------|
| `auv_controller_node`        | 50 Hz T-S fuzzy + allocator + QP loop     |
| `reference_generator_node`   | Publishes `x_ref(t)` on `/auv/reference_state` |
| `test_allocator`             | Offline sanity check (no ROS graph)       |

---

## Run

### One-shot full simulation

```bash
ros2 launch auv_bringup full_simulation.launch.py
```

This starts Gazebo, spawns the AUV at depth 3 m, and launches the
controller + reference generator. After ~3 s startup you should see the
AUV begin cruising forward at ~0.8 m/s.

### Live web dashboard

The `auv_dashboard` package serves a live HTML dashboard that plots
`/auv/virtual_u`, `/auv/tau_des`, `/auv/tau_actual`, `/auv/fault_status`
and the odometry state, with buttons to inject a fault on any channel
through the `/auv/inject_fault` service.

```bash
# Install rosbridge once:
sudo apt install -y ros-humble-rosbridge-server

# Bring up sim + controller + dashboard together:
ros2 launch auv_bringup full_simulation_with_dashboard.launch.py
```

The dashboard opens automatically in your default browser at
`file://.../auv_dashboard/share/web/dashboard.html`. Alternatively start
it by itself:

```bash
ros2 launch auv_dashboard dashboard.launch.py
```

Click a "Fail" button on any of the four thruster tiles to inject an
abrupt fault on that channel and watch the controller compensate in
real time.


### Trajectory tracking

The controller now runs a cascade outer loop on top of the T-S fuzzy inner
loop. Pick a trajectory shape via the `trajectory` parameter in
`auv_control/config/controller.yaml`:

| value         | description                                       |
|---------------|---------------------------------------------------|
| `hold`        | station-keeping at the origin                     |
| `waypoints`   | 4-corner square                                   |
| `lawnmower`   | survey pattern (default for torpedo)              |
| `figure8`     | figure-8 lemniscate                                |
| `circle`      | circle of radius `traj_scale` (default for RexROV)|

Geometry is parameterised by `traj_scale` (m) and `traj_depth` (m).
The outer-loop tuning lives in the same YAML — `kp_surge`, `kp_heave`,
`kp_yaw`, `cruise_speed`, `waypoint_radius`.

The dashboard's top-down trajectory panel renders:

* the planned waypoints (green polyline + dots),
* the recent path of the AUV (blue line),
* the current target waypoint (yellow disc),
* the vehicle position and heading (red triangle).

Fault injection works in tracking mode too — fail any thruster while the
AUV is mid-mission and watch the cascade keep tracking the target while
the allocator routes around the dead actuator.

Sending an external `/auv/reference_state` message switches the
controller out of trajectory mode and into the legacy direct-velocity
reference behaviour. The trajectory is re-engaged on restart.

### Choose which robot body to spawn

Two models are available out of the box:

| `model:=torpedo`   | (default) small 25 kg torpedo AUV, fast |
| `model:=rexrov`    | RexROV-style ROV (1862 kg) using the lightweight STL mesh from the sub_descriptions package |

Example:

```bash
ros2 launch auv_bringup full_simulation_with_dashboard.launch.py model:=rexrov
```

The RexROV case uses a separate YAML (`auv_control/config/controller_rexrov.yaml`)
with thrust limits and buoyancy scaled to the larger vehicle. Neither
model requires any external UUV-simulator packages.

### Reproduce a specific paper figure

To replay one of the paper's scenarios end-to-end (automatic fault at
t = 150 s, automatic shutdown at t = 300 s, with a rosbag recording):

```bash
# Figure 6 (top) — abrupt u1 loss at t=150s
ros2 launch auv_bringup reproduce_paper.launch.py \
    scenario:=fig6_abrupt_thrust  bag_out:=/tmp/run_fig6

# Figure 6 (bottom) — slowly varying u1 degradation
ros2 launch auv_bringup reproduce_paper.launch.py \
    scenario:=fig6_slow_thrust    bag_out:=/tmp/run_fig6slow

# Figure 7 (top) — abrupt u4 (yaw-moment rudder) loss
ros2 launch auv_bringup reproduce_paper.launch.py \
    scenario:=fig7_abrupt_moment  bag_out:=/tmp/run_fig7
```

After the run completes, generate the PNG plots:

```bash
ros2 run auv_control plot_from_bag.py --bag /tmp/run_fig6 --out figs_fig6
# Produces:
#   figs_fig6/fig05_virtual_u.png   (paper Fig 5 — four input channels)
#   figs_fig6/fig06_force.png       (paper Fig 6 — desired vs. actual tau_x)
#   figs_fig6/fig07_moment.png      (paper Fig 7 — desired vs. actual tau_n)
#   figs_fig6/fig_fault_status.png  (f_i over time)
```

### Inject a fault manually

The service `/auv/inject_fault` accepts `thruster_id ∈ {1,2,3,4}` (paper's
`u1..u4`), a target `fault_factor ∈ [0, 1]` (0 = total loss), and either
`abrupt` or `slow` (with a ramp duration). A CLI helper is provided:

```bash
# Total loss of the surge thrusters (paper's Fig 6 "abrupt fault"):
ros2 run auv_control inject_fault_cli.py --id 1 --factor 0.0 --type abrupt

# Slow 30-second degradation of the yaw rudders down to 20%:
ros2 run auv_control inject_fault_cli.py --id 4 --factor 0.2 \
     --type slow --ramp 30.0
```

### Inspect the signals

```bash
# Paper Figure 5 — "Input signal for fuzzy controller"
ros2 topic echo /auv/virtual_u

# Paper Figures 6-7 — desired vs. actual body wrench under fault
ros2 topic echo /auv/tau_des
ros2 topic echo /auv/tau_actual

# Current fault factors
ros2 topic echo /auv/fault_status
```

Or start a plot live:

```bash
ros2 run rqt_plot rqt_plot \
   /auv/virtual_u/data[0] /auv/virtual_u/data[1] \
   /auv/virtual_u/data[2] /auv/virtual_u/data[3]
```

### Offline test (no Gazebo)

```bash
ros2 run auv_control test_allocator
```

Prints the output of the T-S fuzzy controller and the allocator for four
scenarios (healthy / abrupt u1 loss / u1 loss with tight bounds / partial
u3 loss), mirroring the conditions in the paper's Figures 5-11.

---

## Paper → code mapping

| Paper                                                | Code location                                            |
|------------------------------------------------------|----------------------------------------------------------|
| Eq. 1-7 — AUV 6-DoF dynamics                         | **Simulated implicitly** by Gazebo's rigid-body solver + `envForceBodyFrame()` / drag terms in `auv_controller_node.cpp` |
| Eq. 13 — fuzzy defuzzified control law              | `TSFuzzyController::compute()`                           |
| Eq. 18-19 — membership functions                    | `M_t1_low/high`, `M_t2_neg/zero/pos` in `ts_fuzzy.cpp`    |
| Eq. 20-25 — 6 rule gains K₁…K₆                      | `make_gain()` + ctor of `TSFuzzyController`               |
| Eq. 26 — configuration matrix B                     | `build_B()` in `auv_params.hpp`                          |
| Eq. 31-32 — priority matrix W from fault factor     | `ThrustAllocator::set_fault_factors()`                   |
| Eq. 37 — weighted pseudo-inverse solution           | `ThrustAllocator::pseudo_inverse()`                      |
| Eq. 38-42 — saturation + deviation QP formulation   | `ThrustAllocator::allocate()`                            |
| Eq. 43-49 — active-set solver (KKT iteration)       | `ThrustAllocator::solve_qp()`                            |
| Figure 2 — overall flow                             | `AUVController::step()` in `auv_controller_node.cpp`     |
| Section 5.2 — SMC robustifier (Figs. 12-13)         | `SMCRobustifier::compute()` in `smc_robustifier.cpp`     |
| Figure 3 — propulsion layout                        | `auv_description/urdf/auv.urdf.xacro`                    |
| Figures 5-11 — simulation results                   | Reproducible via `test_allocator` and live topics         |

---

## Important simplifications (honest list)

These are deliberate differences from the paper you should know about:

1. **Gain synthesis.** The paper does not publish numerical `K_j` values —
   it only states they come from an LMI on `(A_i + B_i K_j)ᵀ P + P(A_i + B_i K_j) < 0`
   (Eq. 15-17). We instead use a diagonal PD-like template tuned per
   operating point. The *structure* (6 rules, parallel-distribution
   compensation) is preserved; the specific numerical gains will differ
   from whatever the paper's authors solved internally.
2. **Hydrodynamics.** Gazebo Classic does not simulate buoyancy or drag
   natively. We compute both analytically in the controller node from the
   AUV's twist and add them to the body wrench before publishing to the
   `libgazebo_ros_force` plugin. The drag coefficients in `auv_params.hpp`
   are tuned for qualitative realism, not calibrated to a specific vehicle.
3. **Sensor noise / state estimation.** We take `x(t)` directly from
   `libgazebo_ros_p3d` ground truth. The paper's Section 5 simulations
   make the same assumption (they are not testing a state observer).
4. **SMC layer.** The paper's Section 5.2 adds a sliding-mode-control
   layer for robustness (Figs. 12-13, "Inspired by Ref. [30]"). We
   implement this as a boundary-layer SMC robustifier
   (`SMCRobustifier`) summed with the T-S fuzzy output before allocation:
   `u_virtual = u_TS + u_SMC`, with `u_SMC,i = -eta_i * tanh(s_i / phi_i)`
   and a diagonal sliding surface `s = [e_u, e_w, e_q, e_r]` that mirrors
   the per-channel structure of the T-S gains. Tunable via
   `smc_enabled`, `smc_eta`, `smc_phi` in `controller.yaml`; live surface
   values are republished on `/auv/smc_surface`. Set `smc_enabled: false`
   to recover the paper's pure T-S core behavior.

---

## Fork changes log (TE3002B.102 — Team 2)

Iterative fixes layered on top of the upstream snapshot. Each bullet
maps to its commit; see `git log` for the full diffs.

### Controller / algorithmic
- **SMC robustifier layer.** `SMCRobustifier` class summed with the
  T-S fuzzy output before allocation. Diagonal sliding surface, tanh
  boundary layer, per-channel `eta` and `phi`. See "SMC layer" in the
  simplifications section above.
- **B matrix realigned to a torpedo layout.** The original B mixed
  every channel into every wrench axis and collapsed the pitch / yaw
  rows to the same scalar (`u1+u2+u3+u4`). The current B is a clean
  block-diagonal: `u1, u2` are port/stbd surge thrusters (collective →
  `tau_x`, differential → `tau_n`); `u3, u4` are top/bottom heave
  thrusters (collective → `tau_z`, differential → `tau_m`). `tau_y`
  is zeroed (no sway thruster on a torpedo).
- **K matrix rewritten to match the new B.** Each error component
  drives the channel pair that actually produces force on that axis.
  See `ts_fuzzy.cpp::make_gain()`. The SMC sliding-surface map was
  updated in parallel (`ch 0, 1 → e_u`; `ch 2, 3 → e_w`).
- **Allocator Tikhonov regularization.** With `tau_y = 0` the matrix
  `B Winv B^T` is rank-deficient; `LDLT` was silently returning garbage
  and the allocator handed back `u_virtual` unchanged on faults. A
  `1e-9 I` diagonal damping in the pinv, QP warm-start, and active-set
  KKT solves fixes the kernel direction without changing min-norm
  behaviour.
- **Pure-pursuit lookahead.** Goal point slides linearly from `curr_wp`
  toward `next_wp` as `range_to_curr` drops from `waypoint_radius +
  lookahead` to `waypoint_radius`. The pre- and post-advance goal
  points coincide, so `r_ref` is continuous across waypoint
  transitions. Tunable via `lookahead`.
- **Edge-triggered waypoint advance.** Old level-triggered check
  wrapped through the 8-waypoint lawnmower in milliseconds at spawn.
- **`onReference` ignores idle keep-alive.** `reference_generator_node`
  publishes `[0,0,0,0,0]` to keep `/auv/reference_state` alive, which
  was wedging the controller into manual mode forever. Now only
  non-zero references switch to manual override.
- **`w_ref` sign fix.** The original `clamp(-kp_heave * dz, ...)`
  command pushed the AUV UP when the target was deeper. Sign removed.
- **Outer-loop reference slew.** Per-tick rate limits on `u_ref`,
  `w_ref`, `r_ref` so the inner loop doesn't chase a discontinuous
  setpoint at waypoint transitions.
- **Submersion gate on hydrodynamics.** `buoyancy`, `restoring` and
  `drag` are now multiplied by a `[0, 1]` factor that ramps over
  `hull_half_height` around `surface_z`. The AUV no longer flies
  indefinitely once it pops above the water plane.
- **NaN / out-of-bounds guards.** Three layers: drop non-finite odom
  messages; suppress wrench when `|x|` exceeds physical bounds;
  zero the message if any final wrench component is non-finite. The
  simulation never gets permanently corrupted.
- **Auto-recovery after divergence.** When the bounds guard releases,
  snap `wp_idx_` to the closest waypoint, reset `x_ref` to current
  state and zero the wrench LPF, so the AUV resumes the mission
  instead of fighting a stale setpoint from a different operating
  point.
- **Wrench LPF.** First-order low-pass on the published wrench
  (`wrench_tau` parameter, default 0.1 s) so step changes don't excite
  Gazebo's solver.

### Simulation stability
- `buoyancy_force` 247.6 → 250 N (≈ 1.02× `m·g`; near neutral).
- `thrust_max` 50 → 25 N to keep solver inside its stable regime.
- Thruster moment arms `a, b` 0.10 → 0.30 m, so the realized yaw rate
  matches the cascade outer loop's `r_ref`.
- `cruise_speed` 0.6 → 0.3 m/s and `kp_yaw` 1.0 → 1.5 so the AUV can
  actually complete a 90° turn within one segment of the lawnmower.
- `lookahead` 2.0 m default (parameter).
- `auv_bringup`: controller `TimerAction` delay 3.0 s → 0.3 s. The
  three-second gravity-only free fall after spawn was driving the AUV
  into the seabed before the controller ever published a wrench.
- `auv_gazebo`: torpedo spawn `z` `-3` → `-1`; removed an install
  reference to a non-existent `config/` directory that broke
  `colcon build --symlink-install`.

### Dashboard
- Trajectory panel grew a few pixels per `setInterval` tick because
  `resizeTrajCanvas` was writing the canvas size back as inline CSS.
  Fixed via `position: absolute; inset: 0` and a no-op resize when
  the displayed size hasn't changed.

### Container / bring-up notes
On a setup where the host's GPU passthrough is finicky (NVIDIA dGPU
in PRIME on-demand, Intel Arc iGPU that Mesa `iris` doesn't yet
support) the simulation needs three environment overrides on top of
the packaged `docker compose`:
```bash
unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_DATABASE_URI=""
export CYCLONEDDS_URI=file:///tmp/cyclonedds_relaxed.xml   # SocketReceiveBufferSize min="default"
```

---

## Results

Quantitative evidence collected from the iterative long-tests in this
fork. All numbers come from the same `lawnmower` trajectory at depth
`-2 m`, with the AUV spawned at `(0, 0, -1)`.

### Long-test stability progression

Each row is a checkpoint where a fix landed and the sim was re-run
for ~10 min wall time (the software-rendered Mesa stack puts sim time
at roughly `0.15 × real`).

| Checkpoint              | Prints | Bound guards | Recoveries | Max wp |
|-------------------------|-------:|-------------:|-----------:|-------:|
| Upstream (no fixes)     |  40    | n/a (NaN)    | n/a        | 0      |
| + NaN guards            |  40    | 56           | 0          | 1      |
| + B differential signs  |  40    | 49           | 0          | 1      |
| + tau_y zeroed          |  40    | 24           | 0          | 2      |
| + lookahead + w_ref sign|  60    | 14           | 0          | 1*     |
| + K/B alignment         |  60    | 24           | 0          | 2      |
| + auto-recovery         | 100    |  9           | 26         | **6**  |
| + cruise/lookahead tune | 150    | 31           | 116        | 2*     |

`*` means the AUV was bouncing between waypoints near the spawn while
recovering, so the "max wp" counter doesn't fully reflect mission
progress.

Key behavioural changes that are not captured in the table:
- **Depth target reached**: AUV settles at `z = -1.92` before crossing
  `wp[1]` (target `-2`). Previously it bobbed at `z ≈ -0.14` on the
  surface for the entire run.
- **Zero lateral drift on the first segment**: `y` stays at `0.00 m`
  from `wp[0]` to `wp[1]`, where previously the AUV drifted up to
  `+3 m` NE due to the lateral wrench leak through `tau_y`.
- **NaN never propagates**: across every test post-guards, the
  simulation always returned to a finite state; previously a single
  divergence event permanently corrupted Gazebo's model.

### `test_allocator` offline scenarios

Run with `ros2 run auv_control test_allocator`. Output condensed:

| Scenario                             | `u_cmd`                          | `tau_actual`                       | `tau` err |
|--------------------------------------|----------------------------------|------------------------------------|----------:|
| A — healthy                          | `[-0.45, +0.45, 0, 0]`           | `[0, 0, 0, 0, -0.172]`             | 0.000     |
| B — abrupt u1 loss                   | `[ 0,    +0.04, 0, 0]`           | `[0.034, 0, 0, 0, -0.007]`         | 0.168     |
| C — same + tight bounds (QP)         | same as B                        | same as B                          | 0.168     |
| D — partial u3 loss (f3=0.3)         | `[-0.45, +0.45, 0, 0]`           | `[0, 0, 0, 0, -0.172]`             | 0.000     |
| E — aggressive, QP best-effort       | sat. `[+5, +5, +5, +5]`          | `[4.70, 0, 9.39, 0, -0.96]`        | 16.95     |
| F — SMC ON vs OFF, partial u1        | OFF `[9.1, 10.9, 4.0, 4.0]`      | `tau_x` OFF `14.5` / ON `20.2`     | -         |
|                                      | ON  `[13.1, 14.9, 5.0, 6.5]`     | sliding `s=[-0.50, -0.50, -0.20, -0.20]` | -   |

Scenario A is the baseline (controller produces the expected yaw
moment, no error). Scenario D shows a 70 % loss on a single channel
that the allocator absorbs cleanly via the redundancy in its column
pair. Scenario E exercises the active-set QP path under saturation
(status=2 best-effort). Scenario F is the most directly visible
evidence that the new `SMCRobustifier` adds the expected boundary-
layer correction on top of the T-S fuzzy core: ~4 N of forward push
when the surge tracking error sits at `-0.5 m/s`.

The residual on scenario B is the geometric consequence of the new
diagonal-block B (see "Known issues" item 2).

### Dashboard

Live during sim:
- `/auv/inject_fault` service call from the dashboard's **Fail**
  buttons round-trips correctly. Confirmed end-to-end: clicking
  "Fail" on `u1` flips `/auv/fault_status` from `[1,1,1,1]` to
  `[0,1,1,1]` within the dashboard's 100 ms websocket budget.
- The Trajectory panel no longer grows by a few pixels per tick.

---

## Known issues / future work

Things the long-test exposed that still need attention.

1. **Lawnmower never completes a full lap.** The AUV reaches `wp[1]`
   and `wp[2]` cleanly but eventually diverges during the next
   90° turn. Mechanism: as the AUV rotates past 90°, the world-frame
   velocity vector projects into body frame with the opposite sign,
   `e_u` jumps a couple m/s in a single tick, the K matrix saturates
   `u_virtual` to ±25 N, and Gazebo Classic's solver can't absorb the
   resulting wrench step. The auto-recovery + NaN guards keep the
   simulation alive (AUV got as far as `wp[6]` in the most recent
   long-test) but it never tracks the full lap cleanly. A real fix
   would be one of:
   - migrate to Gazebo Garden / Harmonic (better solver);
   - smooth the trajectory with arcs instead of sharp 90° corners;
   - rewrite the inner loop with Coriolis-style decoupling so that
     `e_u` is computed relative to the bearing direction rather than
     a sign-flipping body axis.
2. **Allocator behaviour with the cleaner B is less "redundant".**
   The new diagonal-block B is geometrically honest — `u1` and `u2`
   are the only channels that can generate `tau_x` and `tau_n`. When
   `u1` dies, the constraints `tau_x = 0` and `tau_n ≠ 0` are
   contradictory with only `u2` left, so the weighted-pinv returns
   a small least-squares compromise instead of a full reallocation.
   This is the right math, but it makes the fault-tolerance demo
   less dramatic than the upstream B did. Possible follow-ups:
   add a vectored thrust column to `B` (small off-diagonal coupling)
   so each fault has a graceful fallback path, or accept the new
   geometry and write a more nuanced demo scenario.
3. **`reference_generator_node` is vestigial.** Now that the cascade
   outer loop drives `x_ref` from the trajectory, the reference
   generator only exists to keep `/auv/reference_state` alive at low
   rate. The `onReference` callback ignores its idle messages but the
   node still runs. Either remove it from the launches or repurpose
   it (e.g., to publish a step disturbance during a test scenario).
4. **No FDI.** Faults are still injected manually via the
   `/auv/inject_fault` service (or the dashboard button). A residual-
   based detector that estimates `f_i` online from
   `tau_des - tau_actual` would close the loop and lets the demo
   show the controller noticing a fault on its own. The existing
   `/auv/fault_status` topic already carries the right vector for a
   FDI estimate to replace the manually-injected values.
5. **State observer / sensor noise.** Still using p3d ground truth.
   Gap #3 in the simplifications list — adding an EKF/UKF with
   realistic IMU + DVL noise is a natural follow-on.
6. **Long-test ergonomics.** Bring-up requires four environment
   overrides above; bundle them into `scripts/run_sim.sh` to make it
   one command, and add a `.gitignore` to the workspace root for the
   colcon `build/`, `install/`, `log/` directories.

---

## Troubleshooting

* **`/auv/odom` is silent.** Check that the `libgazebo_ros_p3d` plugin
  loaded: `ros2 topic list | grep auv`. If missing, ensure
  `ros-humble-gazebo-ros-pkgs` is installed and that Gazebo started
  without complaint (`ros2 launch auv_bringup full_simulation.launch.py`
  will print any plugin errors).
* **AUV sinks or floats away.** Adjust `buoyancy_force` in
  `auv_control/config/controller.yaml` — the default is ~1% positive.
* **QP status keeps returning 2.** The QP hit its 40-iteration cap with
  residual KKT violations. This usually means the desired wrench is
  physically infeasible given the remaining healthy actuators —
  widen `thrust_max` or reduce the reference aggressiveness.
* **Controller runs but AUV doesn't respond.** Confirm that `/auv/wrench`
  has subscribers (`ros2 topic info /auv/wrench`) and that the plugin
  name in the URDF matches `libgazebo_ros_force`.

---

## License

Apache-2.0 (same as the original `fuzzy_controller` reference repo bundled
with this task). The paper's intellectual contribution belongs to Zhang
et al.; this codebase is an independent re-implementation for research
and teaching.
