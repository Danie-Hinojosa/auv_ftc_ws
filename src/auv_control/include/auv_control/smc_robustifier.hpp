// ============================================================================
//  auv_control/smc_robustifier.hpp
//
//  Boundary-layer sliding-mode robustifier that sits on top of the T-S fuzzy
//  state-feedback controller. Implements the Section 5.2 SMC extension that
//  Zhang et al. (Sensors 24, 3029) describe but do not include in their core
//  pipeline ("Inspired by Ref. [30]", Figs. 12-13).
//
//      u_total(t) = u_TS(t) + u_SMC(t)
//      u_SMC,i    = -eta_i * tanh(s_i / phi_i)
//      s_i        = e_i  with  e = x - x_ref
//
//  The sliding surface is diagonal to match the per-channel structure of the
//  T-S fuzzy gains (each virtual actuator already targets a single state):
//
//      virtual u   sliding variable        state index
//      ---------   ----------------        -----------
//      u1 surge    s1 = u - u_ref          0
//      u2 heave    s2 = w - w_ref          2
//      u3 pitch    s3 = q - q_ref          3
//      u4 yaw      s4 = r - r_ref          4
//
//  tanh() rather than sign() gives a smooth boundary layer of half-width phi,
//  which preserves the reaching condition outside |s|>phi and removes the
//  high-frequency chattering that pure sign() would inject into the allocator.
// ============================================================================
#ifndef AUV_CONTROL__SMC_ROBUSTIFIER_HPP_
#define AUV_CONTROL__SMC_ROBUSTIFIER_HPP_

#include <array>

#include "auv_control/auv_params.hpp"

namespace auv_control {

class SMCRobustifier {
 public:
  SMCRobustifier();

  // Compute the additive SMC term u_SMC (4-dim) to be summed with the T-S
  // fuzzy output before allocation. When disabled, returns the zero vector
  // (and still records s_ for diagnostics).
  ControlVec compute(const StateVec & x, const StateVec & x_ref) const;

  // Per-channel switching gain (units: same as virtual u, ~N). Larger eta
  // increases robustness to matched uncertainty but also actuator load.
  void set_eta(const std::array<double, 4> & eta) { eta_ = eta; }
  const std::array<double, 4> & eta() const { return eta_; }

  // Per-channel boundary-layer thickness (units: same as the sliding variable,
  // i.e. m/s for surge/heave and rad/s for pitch/yaw rate). Smaller phi
  // pushes behavior toward classical sign() SMC and may chatter.
  void set_phi(const std::array<double, 4> & phi) { phi_ = phi; }
  const std::array<double, 4> & phi() const { return phi_; }

  void set_enabled(bool e) { enabled_ = e; }
  bool enabled() const { return enabled_; }

  // Last sliding-surface vector s (in the same channel order as u_virtual).
  const std::array<double, 4> & last_surface() const { return s_; }

 private:
  // Map a virtual-actuator index (0..3) to the state index it tracks.
  static int state_index_for_channel(int ch);

  std::array<double, 4> eta_ = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> phi_ = {0.05, 0.05, 0.05, 0.05};
  bool enabled_ = false;
  mutable std::array<double, 4> s_{};
};

}  // namespace auv_control

#endif  // AUV_CONTROL__SMC_ROBUSTIFIER_HPP_
