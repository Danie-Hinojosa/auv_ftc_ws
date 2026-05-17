// ============================================================================
//  auv_control/thrust_allocator.cpp
// ============================================================================
#include "auv_control/thrust_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace auv_control {

namespace {
constexpr double kWeightSaturation = 1.0e12;  // stand-in for w_i = +inf
constexpr double kEpsKKT           = 1.0e-6;
constexpr int    kMaxQPIter        = 40;
}  // namespace

// ---------------------------------------------------------------------------
ThrustAllocator::ThrustAllocator(const AllocParams & geom) : geom_(geom) {
  B_ = build_B(geom);
  W_ = Eigen::Matrix<double, 4, 4>::Identity();
  fault_ = {1.0, 1.0, 1.0, 1.0};
}

void ThrustAllocator::set_fault_factors(const std::array<double, 4> & f) {
  fault_ = f;
  W_.setZero();
  for (int i = 0; i < 4; ++i) {
    const double fi = std::clamp(f[i], 0.0, 1.0);
    double wi;
    if (fi <= 1.0e-6) {
      wi = kWeightSaturation;
    } else {
      // Eq. (32):  w_i = exp(1/f_i - 1). Healthy -> 1, severe -> very large.
      wi = std::exp(1.0 / fi - 1.0);
    }
    W_(i, i) = wi;
  }
}

// ---------------------------------------------------------------------------
//  Weighted pseudo-inverse (Eq. 37):
//        T = W^{-1} B^T (B W^{-1} B^T)^{-1} tau
//
//  B has a zero row for tau_y (the torpedo has no sway thruster), which
//  makes M = B Winv B^T rank-deficient. LDLT on a singular matrix returns
//  garbage; the symptom was the allocator handing back u_virtual unchanged
//  no matter how high the fault weights pushed Winv. We add a small
//  Tikhonov term to the kernel direction so the solve stays well-posed.
//  The minimum-norm solution in the row-space of B is unaffected.
// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::pseudo_inverse(const WrenchVec & tau) const {
  const Eigen::Matrix<double, 4, 4> Winv = W_.inverse();
  Eigen::Matrix<double, 5, 5> M = B_ * Winv * B_.transpose();
  M += 1.0e-9 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::Matrix<double, 5, 1> lambda = M.ldlt().solve(tau);
  return Winv * B_.transpose() * lambda;
}

// ---------------------------------------------------------------------------
//  Active-set QP:   min (1/2)||v||^2
//                   s.t.  B v  = tau_des
//                         u_min <= v <= u_max
//
//  Strategy: start from the pseudo-inverse solution. For any violated bound
//  fix v_i to that bound and re-solve the reduced equality-constrained QP
//  in the remaining free variables (KKT system, closed-form solve). Repeat
//  until no bound is violated and all Lagrange multipliers on active bounds
//  have the correct sign.
//
//  This is the practical realisation of Section 4.2, Eqs. (45)-(49), applied
//  to box constraints (the "active set" is just the set of pinned indices).
// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::solve_qp(const WrenchVec & tau_des,
                                     double u_min, double u_max,
                                     bool * converged) const {
  if (converged) *converged = false;

  // Start from the unweighted (plain) pseudo-inverse — the QP objective has
  // no W, so start from the minimum-norm feasible point. Same Tikhonov
  // regularization as pseudo_inverse() to keep the tau_y kernel from
  // breaking LDLT.
  Eigen::Matrix<double, 5, 5> BBT = B_ * B_.transpose();
  BBT += 1.0e-9 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::Matrix<double, 5, 1> lambda = BBT.ldlt().solve(tau_des);
  ControlVec v = B_.transpose() * lambda;

  // Active bounds tracker: 0 = free, -1 = pinned at u_min, +1 = pinned at u_max.
  std::array<int, 4> pin = {0, 0, 0, 0};

  for (int iter = 0; iter < kMaxQPIter; ++iter) {
    // -------- 1) Detect the worst bound violation. ------------------
    double worst_violation = 0.0;
    int    worst_idx       = -1;
    int    worst_sign      = 0;
    for (int i = 0; i < 4; ++i) {
      if (pin[i] != 0) continue;
      if (v(i) > u_max + 1e-9) {
        const double vio = v(i) - u_max;
        if (vio > worst_violation) { worst_violation = vio; worst_idx = i; worst_sign = +1; }
      } else if (v(i) < u_min - 1e-9) {
        const double vio = u_min - v(i);
        if (vio > worst_violation) { worst_violation = vio; worst_idx = i; worst_sign = -1; }
      }
    }

    if (worst_idx < 0) {
      // Primal feasible. Check dual (sign of multipliers on pinned bounds).
      // Multiplier on v_i = +u_max bound is -(gradient entry) when pin[i]=+1.
      // Residual gradient of (1/2)||v||^2 - mu^T (Bv - tau) is v - B^T mu.
      // Solve for mu: from free components, v_free - (B_free)^T mu = 0 => OK by construction.
      // For bound i pinned at +u_max, dual multiplier is (v_i - (B_i)^T mu).
      // If any multiplier has the "wrong" sign we free that bound.
      const Eigen::Matrix<double, 5, 1> mu = lambda;
      bool removed = false;
      for (int i = 0; i < 4; ++i) {
        if (pin[i] == 0) continue;
        const double bi_mu = B_.col(i).dot(mu);
        const double dual  = v(i) - bi_mu;   // grad-of-lagrangian balance
        if (pin[i] == +1 && dual <  -kEpsKKT) { pin[i] = 0; removed = true; break; }
        if (pin[i] == -1 && dual >   kEpsKKT) { pin[i] = 0; removed = true; break; }
      }
      if (!removed) {
        if (converged) *converged = true;
        return v;
      }
    } else {
      pin[worst_idx] = worst_sign;
    }

    // -------- 2) Solve the reduced KKT system. ----------------------
    // Pinned variables are fixed; free variables satisfy:
    //     v_free = (B_free)^T lambda
    //     B_free  v_free  =  tau_des  -  B_pin * v_pin
    // =>  B_free (B_free)^T  lambda  =  tau_des - B_pin * v_pin
    Eigen::Matrix<double, 5, 1> rhs = tau_des;
    for (int i = 0; i < 4; ++i) {
      if (pin[i] == +1) { v(i) = u_max; rhs -= B_.col(i) * u_max; }
      if (pin[i] == -1) { v(i) = u_min; rhs -= B_.col(i) * u_min; }
    }

    // Build B_free with only the free columns.
    std::vector<int> free_idx;
    free_idx.reserve(4);
    for (int i = 0; i < 4; ++i) if (pin[i] == 0) free_idx.push_back(i);

    if (free_idx.empty()) {
      // All pinned — the equality B v = tau_des may not hold. Return v as-is.
      if (converged) *converged = false;
      return v;
    }

    Eigen::MatrixXd Bfree(5, free_idx.size());
    for (std::size_t k = 0; k < free_idx.size(); ++k) {
      Bfree.col(k) = B_.col(free_idx[k]);
    }

    // Solve B_free B_free^T  lambda = rhs  (5x5 SPD for the equality part),
    // Tikhonov-regularized for the tau_y kernel just like the unconstrained
    // pseudo-inverse above.
    Eigen::Matrix<double, 5, 5> M = Bfree * Bfree.transpose();
    M += 1.0e-9 * Eigen::Matrix<double, 5, 5>::Identity();
    lambda = M.ldlt().solve(rhs);

    Eigen::VectorXd vfree = Bfree.transpose() * lambda;
    for (std::size_t k = 0; k < free_idx.size(); ++k) {
      v(free_idx[k]) = vfree(k);
    }
  }

  return v;   // returned with converged=false (hit iteration cap)
}

// ---------------------------------------------------------------------------
ControlVec ThrustAllocator::allocate(const WrenchVec & tau_des,
                                     double u_min, double u_max,
                                     int * status_out) const {
  std::ostringstream rep;

  // Step 1: weighted pseudo-inverse (priority-aware, no constraints).
  ControlVec u = pseudo_inverse(tau_des);

  const bool saturates =
      (u.array() > u_max + 1e-6).any() || (u.array() < u_min - 1e-6).any();

  if (!saturates) {
    if (status_out) *status_out = 0;
    rep << "pinv OK   |u|_inf=" << u.lpNorm<Eigen::Infinity>();
    report_ = rep.str();
    return u;
  }

  // Step 2: QP re-allocation with box constraints.
  bool ok = false;
  ControlVec u_qp = solve_qp(tau_des, u_min, u_max, &ok);

  // Safety clamp — QP should satisfy bounds, but enforce in case of
  // numerical drift.
  for (int i = 0; i < 4; ++i) {
    u_qp(i) = std::clamp(u_qp(i), u_min, u_max);
  }

  if (status_out) *status_out = ok ? 1 : 2;
  rep << (ok ? "QP OK   " : "QP best-effort ")
      << " |u_pinv|_inf=" << u.lpNorm<Eigen::Infinity>()
      << " |u_qp|_inf="   << u_qp.lpNorm<Eigen::Infinity>();
  report_ = rep.str();
  return u_qp;
}

// ---------------------------------------------------------------------------
//  actual_wrench = B * diag(f) * u_cmd
//      Models the paper's fault description (Eq. 33): a partially-failed
//      actuator produces  f_i * u_cmd_i  worth of effective thrust.
// ---------------------------------------------------------------------------
WrenchVec ThrustAllocator::actual_wrench(const ControlVec & u_cmd) const {
  ControlVec u_eff;
  for (int i = 0; i < 4; ++i) u_eff(i) = fault_[i] * u_cmd(i);
  return B_ * u_eff;
}

}  // namespace auv_control
