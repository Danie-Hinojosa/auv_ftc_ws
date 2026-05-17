// ============================================================================
//  auv_control/smc_robustifier.cpp
// ============================================================================
#include "auv_control/smc_robustifier.hpp"

#include <algorithm>
#include <cmath>

namespace auv_control {

int SMCRobustifier::state_index_for_channel(int ch) {
  // Same diagonal mapping as make_gain() in ts_fuzzy.cpp:
  //   ch 0 -> surge   (state u, index 0)
  //   ch 1 -> heave   (state w, index 2)
  //   ch 2 -> pitch   (state q, index 3)
  //   ch 3 -> yaw     (state r, index 4)
  static constexpr int kMap[4] = {0, 2, 3, 4};
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
