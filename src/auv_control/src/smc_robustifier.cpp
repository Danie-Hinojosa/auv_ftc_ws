// ============================================================================
//  auv_control/smc_robustifier.cpp
// ============================================================================
#include "auv_control/smc_robustifier.hpp"

#include <algorithm>
#include <cmath>

namespace auv_control {

int SMCRobustifier::state_index_for_channel(int ch) {
  // Mapping aligned with the physical actuator layout in build_B + the
  // collective/differential K structure in ts_fuzzy.cpp:
  //   ch 0, ch 1 (u1, u2 = port/stbd surge thrusters) -> e_u
  //   ch 2, ch 3 (u3, u4 = top/bottom heave thrusters) -> e_w
  // Both thrusters in a pair share the same sliding surface, so the SMC
  // term injects collective surge / heave force only -- no differential
  // moment contribution (those are the K matrix's job).
  static constexpr int kMap[4] = {0, 0, 2, 2};
  return kMap[ch];
}

SMCRobustifier::SMCRobustifier() = default;

ControlVec SMCRobustifier::compute(const StateVec & x,
                                   const StateVec & x_ref) const {
  ControlVec u_smc = ControlVec::Zero();
  for (int ch = 0; ch < 4; ++ch) {
    const int idx = state_index_for_channel(ch);
    const double err = x(idx) - x_ref(idx);
    s_[ch] = err;
    if (!enabled_) continue;
    // Boundary-layer SMC. tanh saturates at +/-1 outside |s|>~phi,
    // is linear in s inside the layer.
    const double phi = std::max(phi_[ch], 1e-6);
    u_smc(ch) = -eta_[ch] * std::tanh(err / phi);
  }
  return u_smc;
}

}  // namespace auv_control
