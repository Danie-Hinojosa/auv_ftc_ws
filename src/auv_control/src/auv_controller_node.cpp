// ============================================================================
//  auv_control/auv_controller_node.cpp
//
//  Main ROS 2 node implementing the full Figure-2 pipeline of Zhang et al.,
//  Sensors 24, 3029 + a cascade outer loop for trajectory tracking.
//
//  Inner loop  : T-S fuzzy state-feedback on x = [u v w q r]
//  Allocation  : weighted pseudo-inverse + active-set QP under saturation
//  Plant model : tau = B * diag(f) * u_cmd  (fault-effective wrench)
//
//  Outer loop  : position controller. Given a target pose (x*, y*, z*, psi*),
//                produces the velocity reference x_ref for the inner loop.
//                Surge u_ref is proportional to range-to-target capped at
//                cruise speed; yaw r_ref steers toward the target bearing;
//                heave w_ref cancels depth error.
//
//  Topics:
//      /auv/odom              <-  nav_msgs/Odometry   (from libgazebo_ros_p3d)
//      /auv/wrench            ->  geometry_msgs/Wrench (to libgazebo_ros_force)
//      /auv/virtual_u         ->  std_msgs/Float64MultiArray  (4-dim u_cmd)
//      /auv/tau_des           ->  std_msgs/Float64MultiArray  (5-dim)
//      /auv/tau_actual        ->  std_msgs/Float64MultiArray  (5-dim)
//      /auv/fault_status      ->  std_msgs/Float64MultiArray  (4 factors)
//      /auv/reference_state   <-  std_msgs/Float64MultiArray  (5-dim x_ref)
//      /auv/target_pose       ->  geometry_msgs/PoseStamped   (current target)
//      /auv/path              ->  nav_msgs/Path               (recent trail)
//      /auv/inject_fault      <-  service auv_control/srv/InjectFault
// ============================================================================
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "auv_control/srv/inject_fault.hpp"
#include "auv_control/auv_params.hpp"
#include "auv_control/ts_fuzzy.hpp"
#include "auv_control/thrust_allocator.hpp"
#include "auv_control/smc_robustifier.hpp"

using namespace std::chrono_literals;

namespace auv_control {

struct FaultRamp {
  int    idx       = -1;
  double f_start   = 1.0;
  double f_target  = 1.0;
  double t_start   = 0.0;
  double ramp_sec  = 0.0;
  bool   active    = false;
};

// One waypoint in body-frame position.
struct Waypoint { double x, y, z, yaw; };

class AUVController : public rclcpp::Node {
 public:
  AUVController()
  : Node("auv_controller"),
    allocator_(geom_),
    fuzzy_() {
    // ---- Inner loop & physics parameters --------------------------------
    declare_parameter("control_rate_hz",  50.0);
    declare_parameter("thrust_max",       veh_.thrust_max);
    declare_parameter("thrust_min",       veh_.thrust_min);
    declare_parameter("buoyancy_force",   veh_.buoyancy_force);
    declare_parameter("metacentric_height", 0.05);  // m, GM positive = stable
    // Cascade outer-loop gains.
    declare_parameter("kp_surge",         0.6);
    declare_parameter("kp_heave",         0.5);
    declare_parameter("kp_yaw",           1.0);
    declare_parameter("cruise_speed",     0.6);
    declare_parameter("waypoint_radius",  0.8);
    // Trajectory generator.
    declare_parameter<std::string>("trajectory", "lawnmower");
    declare_parameter("traj_scale",       6.0);
    declare_parameter("traj_depth",       -3.0);

    // Water/air interface model. Above the surface the AUV gets no buoyancy,
    // no hydrodynamic drag and no restoring moment; without this the
    // controller would keep flying it indefinitely once it pops up.
    declare_parameter("surface_z", 0.0);
    declare_parameter("hull_half_height", 0.15);

    // Sliding-mode robustifier (Section 5.2 of Zhang et al.). Diagonal SMC
    // layered on top of the T-S fuzzy output. eta/phi per virtual channel
    // [surge, heave, pitch, yaw]. Defaults are conservative — bump eta
    // upward to absorb larger matched uncertainty / disturbance bounds.
    declare_parameter("smc_enabled", true);
    declare_parameter<std::vector<double>>(
        "smc_eta", std::vector<double>{4.0, 4.0, 1.0, 2.5});
    declare_parameter<std::vector<double>>(
        "smc_phi", std::vector<double>{0.08, 0.08, 0.05, 0.05});

    double rate_hz;
    get_parameter("control_rate_hz", rate_hz);
    get_parameter("thrust_max",      veh_.thrust_max);
    get_parameter("thrust_min",      veh_.thrust_min);
    get_parameter("buoyancy_force",  veh_.buoyancy_force);
    get_parameter("metacentric_height", GM_);
    get_parameter("kp_surge", kp_surge_);
    get_parameter("kp_heave", kp_heave_);
    get_parameter("kp_yaw",   kp_yaw_);
    get_parameter("cruise_speed",    cruise_speed_);
    get_parameter("waypoint_radius", waypoint_radius_);

    auto tname = get_parameter("trajectory").as_string();
    double scale, depth;
    get_parameter("traj_scale", scale);
    get_parameter("traj_depth", depth);
    buildTrajectory(tname, scale, depth);

    get_parameter("surface_z",        surface_z_);
    get_parameter("hull_half_height", hull_half_height_);

    {
      bool smc_on;
      get_parameter("smc_enabled", smc_on);
      smc_.set_enabled(smc_on);
      const auto eta_v = get_parameter("smc_eta").as_double_array();
      const auto phi_v = get_parameter("smc_phi").as_double_array();
      std::array<double, 4> eta{}, phi{};
      for (int i = 0; i < 4; ++i) {
        eta[i] = (i < static_cast<int>(eta_v.size())) ? eta_v[i] : 0.0;
        phi[i] = (i < static_cast<int>(phi_v.size())) ? phi_v[i] : 0.05;
      }
      smc_.set_eta(eta);
      smc_.set_phi(phi);
    }

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&AUVController::onOdom, this, std::placeholders::_1));
    sub_ref_  = create_subscription<std_msgs::msg::Float64MultiArray>(
      "reference_state", 10,
      std::bind(&AUVController::onReference, this, std::placeholders::_1));

    pub_wrench_ = create_publisher<geometry_msgs::msg::Wrench>("wrench", 10);
    pub_u_      = create_publisher<std_msgs::msg::Float64MultiArray>("virtual_u", 10);
    pub_taud_   = create_publisher<std_msgs::msg::Float64MultiArray>("tau_des", 10);
    pub_taua_   = create_publisher<std_msgs::msg::Float64MultiArray>("tau_actual", 10);
    pub_fault_  = create_publisher<std_msgs::msg::Float64MultiArray>("fault_status", 10);
    pub_target_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    pub_path_   = create_publisher<nav_msgs::msg::Path>("path", 10);
    pub_traj_   = create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(1).transient_local());
    pub_smc_    = create_publisher<std_msgs::msg::Float64MultiArray>("smc_surface", 10);

    srv_fault_ = create_service<auv_control::srv::InjectFault>(
      "inject_fault",
      std::bind(&AUVController::onInjectFault, this,
                std::placeholders::_1, std::placeholders::_2));

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0));
    timer_     = create_wall_timer(period, std::bind(&AUVController::step, this));
    // Publish the (static) trajectory once per second so late subscribers get it.
    traj_timer_ = create_wall_timer(1s, [this](){ publishTrajectory(); });

    RCLCPP_INFO(get_logger(),
      "auv_controller up: rate=%.1f Hz, thrust=[%+.1f,%+.1f] N, buoyancy=%.1f N, "
      "trajectory='%s' (%zu wps), smc=%s eta=[%.2f %.2f %.2f %.2f]",
      rate_hz, veh_.thrust_min, veh_.thrust_max, veh_.buoyancy_force,
      tname.c_str(), trajectory_.size(),
      smc_.enabled() ? "on" : "off",
      smc_.eta()[0], smc_.eta()[1], smc_.eta()[2], smc_.eta()[3]);
  }

 private:
  // ===== Callbacks ======================================================
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    have_odom_ = true;
    odom_count_++;

    // Body-frame twist (REP-105: child_frame_id = base_link).
    x_(0) = msg->twist.twist.linear.x;
    x_(1) = msg->twist.twist.linear.y;
    x_(2) = msg->twist.twist.linear.z;
    x_(3) = msg->twist.twist.angular.y;
    x_(4) = msg->twist.twist.angular.z;
    p_roll_ = msg->twist.twist.angular.x;

    // World-frame pose.
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
  }

  void onReference(const std_msgs::msg::Float64MultiArray::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_);
    // The reference_generator_node publishes an idle [0,0,0,0,0] at low rate
    // to keep the topic alive; that should NOT switch us into manual mode and
    // wipe out the cascade outer loop. Only treat a message as a real override
    // when at least one component is non-zero.
    bool nonzero = false;
    for (std::size_t i = 0; i < m->data.size() && i < 5; ++i) {
      if (std::abs(m->data[i]) > 1e-6) { nonzero = true; break; }
    }
    if (!nonzero) return;
    manual_ref_ = true;
    for (std::size_t i = 0; i < m->data.size() && i < 5; ++i) {
      x_ref_(i) = m->data[i];
    }
  }

  void onInjectFault(
      const std::shared_ptr<auv_control::srv::InjectFault::Request>  req,
            std::shared_ptr<auv_control::srv::InjectFault::Response> rsp) {
    std::lock_guard<std::mutex> lk(mtx_);
    const int id = req->thruster_id;
    if (id < 1 || id > 4) {
      rsp->accepted = false;
      rsp->message  = "thruster_id must be in [1, 4]";
      return;
    }
    FaultRamp r;
    r.idx      = id - 1;
    r.f_start  = fault_[r.idx];
    r.f_target = std::clamp(req->fault_factor, 0.0, 1.0);
    r.t_start  = now().seconds();
    r.ramp_sec = (req->fault_type == "abrupt") ? 0.0 : std::max(req->ramp_seconds, 0.0);
    r.active   = true;
    ramps_[r.idx] = r;
    rsp->accepted = true;
    rsp->message  = "fault injection scheduled";
    RCLCPP_WARN(get_logger(),
      "FAULT INJECTED on u%d : %.2f -> %.2f over %.1fs (%s)",
      id, r.f_start, r.f_target, r.ramp_sec, req->fault_type.c_str());
  }

  // ===== Main 50 Hz loop =================================================
  void step() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_odom_) return;

    // 1) Faults / priority matrix.
    updateFaults();
    allocator_.set_fault_factors(fault_);

    // 2) OUTER LOOP — pick current waypoint, compute body-frame velocity ref.
    if (!manual_ref_) updateReferenceFromTrajectory();

    // 3) INNER LOOP — T-S fuzzy state feedback + SMC robustifier.
    //    u_virtual = u_TS + u_SMC. The SMC term is a per-channel boundary-
    //    layer term that absorbs matched uncertainty (drag mismatch, buoyancy
    //    estimation error, unmodeled currents). When disabled in config it
    //    contributes exactly zero — the loop reduces to the paper's core.
    const ControlVec u_ts      = fuzzy_.compute(x_, x_ref_);
    const ControlVec u_smc     = smc_.compute(x_, x_ref_);
    const ControlVec u_virtual = u_ts + u_smc;
    const WrenchVec  tau_des   = allocator_.B() * u_virtual;

    int status = 0;
    const ControlVec u_cmd =
        allocator_.allocate(tau_des, veh_.thrust_min, veh_.thrust_max, &status);
    const WrenchVec tau_actual = allocator_.actual_wrench(u_cmd);

    // 4) Hydrodynamics (drag + restoring + buoyancy), gated by submersion.
    //    Gazebo Classic doesn't simulate the air/water interface, so without
    //    this gate the controller keeps applying full buoyancy + drag even
    //    when the AUV pops above the surface, leaving it indefinitely
    //    floating in mid-air. We scale every wet-only term by a 0..1 factor
    //    that linearly transitions across the hull around the surface plane.
    const double sub = submersion(pos_z_);
    const Eigen::Vector3d buoy_body = sub * buoyancyBodyFrame();
    const Eigen::Vector3d restore_M = sub * restoringMomentBodyFrame();
    const Eigen::Vector3d drag_force = -sub * Eigen::Vector3d(
        veh_.drag_u * x_(0) + veh_.drag_uu * std::abs(x_(0)) * x_(0),
        veh_.drag_v * x_(1) + veh_.drag_vv * std::abs(x_(1)) * x_(1),
        veh_.drag_w * x_(2) + veh_.drag_ww * std::abs(x_(2)) * x_(2));
    const Eigen::Vector3d drag_moment = -sub * Eigen::Vector3d(
        veh_.drag_p * p_roll_,
        veh_.drag_q * x_(3) + veh_.drag_qq * std::abs(x_(3)) * x_(3),
        veh_.drag_r * x_(4) + veh_.drag_rr * std::abs(x_(4)) * x_(4));

    // 5) Compose final body-frame wrench. Note that we publish the FULL
    //    buoyancy force in body frame, NOT (buoyancy - mass*g).  Gazebo
    //    applies gravity itself in world frame; if we subtracted mass*g
    //    here too we would double-count and the AUV would sink to the
    //    seabed (the symptom we observed in earlier runs).
    geometry_msgs::msg::Wrench w;
    w.force.x  = tau_actual(0) + buoy_body.x()  + drag_force.x();
    w.force.y  = tau_actual(1) + buoy_body.y()  + drag_force.y();
    w.force.z  = tau_actual(2) + buoy_body.z()  + drag_force.z();
    w.torque.x = restore_M.x() + drag_moment.x();
    w.torque.y = tau_actual(3) + restore_M.y() + drag_moment.y();
    w.torque.z = tau_actual(4) + restore_M.z() + drag_moment.z();
    pub_wrench_->publish(w);

    // 6) Diagnostics.
    publishVec(pub_u_,     {u_cmd(0),     u_cmd(1),     u_cmd(2),     u_cmd(3)});
    publishVec(pub_taud_,  {tau_des(0),   tau_des(1),   tau_des(2),   tau_des(3), tau_des(4)});
    publishVec(pub_taua_,  {tau_actual(0),tau_actual(1),tau_actual(2),tau_actual(3),tau_actual(4)});
    publishVec(pub_fault_, {fault_[0], fault_[1], fault_[2], fault_[3]});
    const auto & s = smc_.last_surface();
    publishVec(pub_smc_,   {s[0], s[1], s[2], s[3]});
    publishTargetPose();
    appendPath();

    if ((tick_++ % 100) == 0) {
      RCLCPP_INFO(get_logger(),
        "pos=(%+.2f, %+.2f, %+.2f) yaw=%+.2f  "
        "x=[u=%+.2f w=%+.2f r=%+.2f]  ref=[u=%+.2f w=%+.2f r=%+.2f]  "
        "u=[%+5.2f %+5.2f %+5.2f %+5.2f]  wp=%zu/%zu  status=%d",
        pos_x_, pos_y_, pos_z_, yaw_,
        x_(0), x_(2), x_(4), x_ref_(0), x_ref_(2), x_ref_(4),
        u_cmd(0), u_cmd(1), u_cmd(2), u_cmd(3),
        wp_idx_, trajectory_.size(), status);
    }
  }

  // ===== Trajectory generator ============================================
  void buildTrajectory(const std::string & name, double scale, double depth) {
    trajectory_.clear();
    if (name == "waypoints") {
      trajectory_ = {
        { 5.0,  0.0, depth, 0.0},
        { 5.0,  5.0, depth, 1.5708},
        { 0.0,  5.0, depth, 3.14159},
        { 0.0,  0.0, depth, -1.5708},
      };
    } else if (name == "lawnmower") {
      // Survey pattern: 4 east-west legs at increasing northing.
      const double L = scale;
      const double dy = scale * 0.5;
      for (int row = 0; row < 4; ++row) {
        const double y = row * dy;
        if (row % 2 == 0) {
          trajectory_.push_back({0.0, y, depth, 0.0});
          trajectory_.push_back({L,   y, depth, 0.0});
        } else {
          trajectory_.push_back({L,   y, depth, 3.14159});
          trajectory_.push_back({0.0, y, depth, 3.14159});
        }
      }
    } else if (name == "figure8") {
      const int N = 32;
      for (int i = 0; i < N; ++i) {
        const double t = 2 * 3.14159 * i / double(N);
        const double x =  scale * std::sin(t);
        const double y =  scale * std::sin(t) * std::cos(t);
        trajectory_.push_back({x, y, depth, 0.0});
      }
      trajectory_.push_back(trajectory_.front());
    } else if (name == "circle") {
      const int N = 24;
      for (int i = 0; i < N; ++i) {
        const double t = 2 * 3.14159 * i / double(N);
        trajectory_.push_back({
          scale * std::cos(t), scale * std::sin(t), depth, t + 1.5708});
      }
      trajectory_.push_back(trajectory_.front());
    } else {
      // "hold" or unknown — single waypoint at origin/depth.
      trajectory_ = {{0.0, 0.0, depth, 0.0}};
    }
    wp_idx_ = 0;
  }

  // Pure-pursuit-style outer loop: choose the active waypoint, derive a
  // body-frame velocity reference (u, w, r) for the T-S fuzzy controller.
  void updateReferenceFromTrajectory() {
    if (trajectory_.empty()) return;
    const Waypoint & wp = trajectory_[wp_idx_];

    const double dx = wp.x - pos_x_;
    const double dy = wp.y - pos_y_;
    const double dz = wp.z - pos_z_;
    const double range_xy = std::hypot(dx, dy);

    // Edge-triggered waypoint advance: only when the AUV crosses INTO the
    // capture radius (transition from outside to inside). The old level-
    // triggered check (advance every tick while range < radius) wrapped
    // through the 8-waypoint lawnmower in ~1 s because the AUV starts
    // already inside wp[0]'s radius and the controller's wall-clock step()
    // fires faster than the AUV can leave the radius.
    const bool inside = (range_xy < waypoint_radius_);
    if (inside && !was_inside_wp_radius_) {
      wp_idx_ = (wp_idx_ + 1) % trajectory_.size();
    }
    was_inside_wp_radius_ = inside;

    const double bearing = std::atan2(dy, dx);
    double yaw_err = bearing - yaw_;
    while (yaw_err >  3.14159) yaw_err -= 2 * 3.14159;
    while (yaw_err < -3.14159) yaw_err += 2 * 3.14159;

    // u_ref: cruise speed scaled down when we have a big yaw error.
    const double align = std::cos(yaw_err);
    const double u_ref = std::clamp(
        kp_surge_ * range_xy * std::max(align, 0.0),
        0.0, cruise_speed_);

    // Heave reference proportional to depth error (saturated).
    const double w_ref = std::clamp(-kp_heave_ * dz, -0.4, 0.4);

    // Yaw rate reference proportional to bearing error.
    const double r_ref = std::clamp(kp_yaw_ * yaw_err, -0.6, 0.6);

    x_ref_(0) = u_ref;
    x_ref_(1) = 0.0;
    x_ref_(2) = w_ref;
    x_ref_(3) = 0.0;
    x_ref_(4) = r_ref;
  }

  // ===== Hydrodynamic helpers ============================================
  // Fraction of the AUV that is submerged: 1.0 fully under, 0.0 fully above
  // the water surface (world frame, z=0). Linear ramp across hull_half_height_
  // on either side of the surface plane gives a smooth handoff so the wrench
  // doesn't switch discontinuously when the AUV crosses the interface.
  double submersion(double z_world) const {
    const double dz = z_world - surface_z_;  // dz > 0 means above water
    if (dz <= -hull_half_height_) return 1.0;
    if (dz >=  hull_half_height_) return 0.0;
    return 0.5 - 0.5 * dz / hull_half_height_;
  }

  // Body-frame buoyancy: world-frame  +B*z_world  rotated into the body.
  Eigen::Vector3d buoyancyBodyFrame() const {
    const double cr = std::cos(roll_),  sr = std::sin(roll_);
    const double cp = std::cos(pitch_), sp = std::sin(pitch_);
    const double cy = std::cos(yaw_),   sy = std::sin(yaw_);
    Eigen::Matrix3d Rwb;
    Rwb <<  cy*cp,                sy*cp,               -sp,
            cy*sp*sr - sy*cr,     sy*sp*sr + cy*cr,     cp*sr,
            cy*sp*cr + sy*sr,     sy*sp*cr - cy*sr,     cp*cr;
    return Rwb * Eigen::Vector3d(0.0, 0.0, veh_.buoyancy_force);
  }

  // Restoring moment: when the AUV pitches/rolls, the buoyancy at center-
  // of-buoyancy (above CoG by GM) creates a righting torque  -B*GM*sin(theta).
  // Approximation:  M_roll  = -B * GM * sin(roll)
  //                 M_pitch = -B * GM * sin(pitch)
  Eigen::Vector3d restoringMomentBodyFrame() const {
    const double k = veh_.buoyancy_force * GM_;
    return Eigen::Vector3d(-k * std::sin(roll_), -k * std::sin(pitch_), 0.0);
  }

  // ===== Diagnostics publishers =========================================
  void publishTargetPose() {
    if (trajectory_.empty()) return;
    const Waypoint & wp = trajectory_[wp_idx_];
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp    = now();
    p.pose.position.x = wp.x;
    p.pose.position.y = wp.y;
    p.pose.position.z = wp.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, wp.yaw);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    pub_target_->publish(p);
  }

  void publishTrajectory() {
    if (trajectory_.empty()) return;
    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp    = now();
    for (const auto & wp : trajectory_) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = wp.x;
      p.pose.position.y = wp.y;
      p.pose.position.z = wp.z;
      path.poses.push_back(p);
    }
    pub_traj_->publish(path);
  }

  void appendPath() {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp    = now();
    p.pose.position.x = pos_x_;
    p.pose.position.y = pos_y_;
    p.pose.position.z = pos_z_;
    path_buf_.push_back(p);
    if (path_buf_.size() > 600) path_buf_.pop_front();

    if ((tick_ % 5) == 0) {     // 10 Hz output is plenty for a trail
      nav_msgs::msg::Path msg;
      msg.header.frame_id = "world";
      msg.header.stamp    = now();
      msg.poses.assign(path_buf_.begin(), path_buf_.end());
      pub_path_->publish(msg);
    }
  }

  // ===== Helpers ========================================================
  void updateFaults() {
    const double t = now().seconds();
    for (int i = 0; i < 4; ++i) {
      auto & r = ramps_[i];
      if (!r.active) continue;
      if (r.ramp_sec <= 1e-6) {
        fault_[i] = r.f_target;
        r.active  = false;
      } else {
        const double s = std::clamp((t - r.t_start) / r.ramp_sec, 0.0, 1.0);
        fault_[i] = r.f_start + s * (r.f_target - r.f_start);
        if (s >= 1.0) r.active = false;
      }
    }
  }

  template <typename PubT>
  void publishVec(PubT & pub, std::vector<double> v) {
    std_msgs::msg::Float64MultiArray m;
    m.data = std::move(v);
    pub->publish(m);
  }

  // ===== Members ========================================================
  VehicleParams    veh_{};
  AllocParams      geom_{};
  ThrustAllocator  allocator_;
  TSFuzzyController fuzzy_;
  SMCRobustifier   smc_;

  StateVec x_     = StateVec::Zero();
  StateVec x_ref_ = StateVec::Zero();
  double   p_roll_ = 0.0;
  double   roll_  = 0.0, pitch_ = 0.0, yaw_ = 0.0;
  double   pos_x_ = 0.0, pos_y_ = 0.0, pos_z_ = 0.0;
  bool     have_odom_  = false;
  bool     manual_ref_ = false;
  std::size_t odom_count_ = 0;

  // Outer loop / trajectory.
  double GM_ = 0.05;
  double kp_surge_ = 0.6, kp_heave_ = 0.5, kp_yaw_ = 1.0;
  double cruise_speed_ = 0.6, waypoint_radius_ = 0.8;
  double surface_z_ = 0.0, hull_half_height_ = 0.15;
  std::vector<Waypoint> trajectory_;
  std::size_t wp_idx_ = 0;
  bool was_inside_wp_radius_ = false;
  std::deque<geometry_msgs::msg::PoseStamped> path_buf_;

  std::array<double, 4>    fault_ = {1.0, 1.0, 1.0, 1.0};
  std::array<FaultRamp, 4> ramps_{};

  std::mutex mtx_;
  std::size_t tick_ = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr          pub_wrench_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_u_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_taud_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_taua_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_fault_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pub_target_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 pub_traj_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_smc_;
  rclcpp::Service<auv_control::srv::InjectFault>::SharedPtr         srv_fault_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
  rclcpp::TimerBase::SharedPtr                                      traj_timer_;
};

}  // namespace auv_control

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_control::AUVController>());
  rclcpp::shutdown();
  return 0;
}
