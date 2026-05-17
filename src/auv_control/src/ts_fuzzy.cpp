// ============================================================================
//  auv_control/ts_fuzzy.cpp
// ============================================================================
#include "auv_control/ts_fuzzy.hpp"

#include <cmath>

namespace auv_control {

// ---------------------------------------------------------------------------
//  Membership functions — exactly Eqs. (18), (19) of the paper.
// ---------------------------------------------------------------------------
double TSFuzzyController::M_t1_low (double t1) { return (2.0 + std::sin(t1)) / 5.0; }
double TSFuzzyController::M_t1_high(double t1) { return (3.0 - std::sin(t1)) / 5.0; }
double TSFuzzyController::M_t2_neg (double t2) { return (std::cos(t2) + 2.0) / 5.0; }
double TSFuzzyController::M_t2_zero(double t2) { return (std::sin(t2) + 3.0) / 5.0; }
double TSFuzzyController::M_t2_pos (double t2) {
  return (-std::sin(t2) - std::cos(t2) + 5.0) / 5.0;
}

// ---------------------------------------------------------------------------
//  Gain-matrix synthesis.
//
//  The paper's K_j are not published. Each rule targets a different operating
//  point, so we keep the same structural template but bias each K_j for its
//  local linearization (low speed ~ higher yaw authority, high speed ~ softer
//  yaw gain to prevent oscillation). State order: [u v w q r].
//
//  Physical actuator mapping (matches build_B in auv_params.hpp):
//    u1, u2 = port / stbd surge thrusters. Collective tracks e_u (surge),
//             differential tracks e_r (yaw rate).
//    u3, u4 = top / bottom heave thrusters. Collective tracks e_w (heave),
//             differential tracks e_q (pitch rate).
//
//  The original gains were "diagonal" in name only -- each error went to a
//  single channel that didn't actually produce force on the relevant axis,
//  so the closed loop was working against itself. Splitting each axis's
//  error across both channels in its physical pair fixes that.
// ---------------------------------------------------------------------------
static Gain make_gain(double ku, double kw, double kq, double kr) {
  Gain K;
  K.setZero();
  // Surge pair (u1, u2): both drive surge collectively, differential is
  // what generates yaw moment.
  K(0, 0) =  ku;   K(0, 4) =  kr;
  K(1, 0) =  ku;   K(1, 4) = -kr;
  // Heave pair (u3, u4): both drive heave collectively, differential is
  // what generates pitch moment.
  K(2, 2) =  kw;   K(2, 3) =  kq;
  K(3, 2) =  kw;   K(3, 3) = -kq;
  return K;
}

TSFuzzyController::TSFuzzyController() {
  //                       ku    kw    kq    kr     (units: N per m/s or N*m per rad/s)
  K_[0] = make_gain( 18.0, 20.0, 3.0, 10.0);   // Rule 1: theta1=0.5, theta2=-0.1
  K_[1] = make_gain( 18.0, 20.0, 3.0,  8.0);   // Rule 2: theta1=0.5, theta2= 0.0
  K_[2] = make_gain( 18.0, 20.0, 3.0, 10.0);   // Rule 3: theta1=0.5, theta2= 0.1
  K_[3] = make_gain( 22.0, 20.0, 4.0,  9.0);   // Rule 4: theta1=1.0, theta2=-0.1
  K_[4] = make_gain( 22.0, 20.0, 4.0,  7.0);   // Rule 5: theta1=1.0, theta2= 0.0
  K_[5] = make_gain( 22.0, 20.0, 4.0,  9.0);   // Rule 6: theta1=1.0, theta2= 0.1
}

// ---------------------------------------------------------------------------
//  compute() — Eq. (13)
// ---------------------------------------------------------------------------
ControlVec TSFuzzyController::compute(const StateVec & x,
                                      const StateVec & x_ref) const {
  const double th1 = x(0);   // theta1 = surge speed u
  const double th2 = x(4);   // theta2 = yaw rate r

  // Rule firing strengths omega_i = product of membership functions for
  // theta1 and theta2.  (Eq. 11 normalises them into mu_i.)
  const double m1l = M_t1_low (th1);
  const double m1h = M_t1_high(th1);
  const double m2n = M_t2_neg (th2);
  const double m2z = M_t2_zero(th2);
  const double m2p = M_t2_pos (th2);

  std::array<double, 6> omega = {
    m1l * m2n,  // R1: theta1=0.5, theta2=-0.1
    m1l * m2z,  // R2: theta1=0.5, theta2= 0.0
    m1l * m2p,  // R3: theta1=0.5, theta2= 0.1
    m1h * m2n,  // R4: theta1=1.0, theta2=-0.1
    m1h * m2z,  // R5: theta1=1.0, theta2= 0.0
    m1h * m2p,  // R6: theta1=1.0, theta2= 0.1
  };

  double denom = 0.0;
  for (double w : omega) denom += w;
  if (denom < 1e-9) denom = 1.0;          // numerical floor

  // Defuzzify: weighted sum of state-feedback contributions.
  ControlVec u = ControlVec::Zero();
  const StateVec e = x - x_ref;
  for (std::size_t j = 0; j < 6; ++j) {
    mu_[j] = omega[j] / denom;
    // State feedback is NEGATIVE of (K * error) — push the state toward
    // the reference (Lyapunov proof in Section 2.2 assumed u = K x with
    // K chosen so that A+BK is Hurwitz; equivalently -K on the error).
    u.noalias() -= mu_[j] * K_[j] * e;
  }
  return u;
}

}  // namespace auv_control
