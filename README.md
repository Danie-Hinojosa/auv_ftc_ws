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
