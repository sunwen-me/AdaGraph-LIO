// Unit tests for basic/Manifold.h math:
//  - Gamma_2 closed form vs numerical integration (Simpson) of
//    Gamma_2(phi) = int_0^1 (1-u) * Exp(u*phi) du
//  - SO3 / SE3 Exp/Log roundtrips, including theta near pi
//  - free-function Log(R) stability near pi
//  - SE3::update()/updateRhs() keep T_/R_/t_ consistent

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "basic/Manifold.h"

namespace {

using BASIC::M3;
using BASIC::M4;
using BASIC::SE3;
using BASIC::SO3;
using BASIC::V3;
using BASIC::V6;

using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;

// Simpson integration of Gamma_2(phi) = int_0^1 (1-u) Exp(u*phi) du.
// This identity follows from the series Gamma_2 = sum_n hat(phi)^n / (n+2)!.
M3d numericalGamma2(const V3d& phi) {
  const int N = 2000;  // even
  const double h = 1.0 / N;
  auto f = [&phi](double u) -> M3d {
    return (1.0 - u) * BASIC::Exp(V3d(u * phi));
  };
  M3d sum = f(0.0) + f(1.0);
  for (int i = 1; i < N; ++i) {
    sum += ((i % 2 == 1) ? 4.0 : 2.0) * f(i * h);
  }
  return (h / 3.0) * sum;
}

double maxAbsDiff(const M3d& a, const M3d& b) {
  return (a - b).cwiseAbs().maxCoeff();
}

std::vector<V3d> randomUnitAxes(int n, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::vector<V3d> axes;
  axes.reserve(n);
  while (static_cast<int>(axes.size()) < n) {
    V3d v(gauss(rng), gauss(rng), gauss(rng));
    if (v.norm() > 1e-3) axes.push_back(v.normalized());
  }
  return axes;
}

TEST(Gamma2Test, MatchesNumericalIntegration) {
  const std::vector<double> thetas = {1.01e-5, 1e-4, 1e-3, 1e-2, 0.1,
                                      0.5,     1.0,  2.0,  3.0,
                                      M_PI - 1e-3, M_PI, 4.0};
  const auto axes = randomUnitAxes(5, 42u);
  for (const auto& axis : axes) {
    for (double theta : thetas) {
      const V3d phi = axis * theta;
      const M3d closed = BASIC::Gamma_2<double>(phi);
      const M3d numeric = numericalGamma2(phi);
      EXPECT_LT(maxAbsDiff(closed, numeric), 1e-9)
          << "theta=" << theta << " axis=" << axis.transpose();
    }
  }
}

TEST(Gamma2Test, SmallAngleBranchMatchesNumericalIntegration) {
  // Below the 1e-5 switch the series branch is used; its truncation error is
  // O(theta^2 / 24) which is < 5e-12 for theta <= 1e-5.
  const std::vector<double> thetas = {0.0, 1e-9, 1e-7, 5e-6, 9.9e-6};
  const auto axes = randomUnitAxes(3, 7u);
  for (const auto& axis : axes) {
    for (double theta : thetas) {
      const V3d phi = axis * theta;
      const M3d closed = BASIC::Gamma_2<double>(phi);
      const M3d numeric = numericalGamma2(phi);
      EXPECT_LT(maxAbsDiff(closed, numeric), 1e-10) << "theta=" << theta;
    }
  }
}

TEST(Gamma2Test, ContinuousAcrossBranchSwitch) {
  const auto axes = randomUnitAxes(3, 11u);
  for (const auto& axis : axes) {
    const M3d below = BASIC::Gamma_2<double>(V3d(axis * 0.999e-5));
    const M3d above = BASIC::Gamma_2<double>(V3d(axis * 1.001e-5));
    EXPECT_LT(maxAbsDiff(below, above), 1e-8);
  }
}

TEST(Gamma2Test, ZeroAngleIsHalfIdentity) {
  const M3d g2 = BASIC::Gamma_2<double>(V3d::Zero());
  EXPECT_LT(maxAbsDiff(g2, 0.5 * M3d::Identity()), 1e-15);
}

// ---------------------------------------------------------------------------
// free-function Log(R), double precision, incl. theta near pi
// ---------------------------------------------------------------------------

TEST(LogTest, RecoversRotationVectorNearPi) {
  const std::vector<double> thetas = {M_PI - 1e-3, M_PI - 1e-5, M_PI - 1e-8};
  const auto axes = randomUnitAxes(5, 21u);
  for (const auto& axis : axes) {
    for (double theta : thetas) {
      const V3d phi = axis * theta;
      const M3d R = BASIC::Exp(V3d(phi));
      const V3d w = BASIC::Log(R);
      EXPECT_LT((w - phi).norm(), 1e-9)
          << "theta=" << theta << " axis=" << axis.transpose();
    }
  }
}

TEST(LogTest, ExactPiRoundtrip) {
  // At exactly pi the sign of the axis is not unique; require the roundtrip
  // Exp(Log(R)) == R and |Log(R)| == pi.
  const auto axes = randomUnitAxes(5, 33u);
  for (const auto& axis : axes) {
    const M3d R = BASIC::Exp(V3d(axis * M_PI));
    const V3d w = BASIC::Log(R);
    EXPECT_NEAR(w.norm(), M_PI, 1e-9);
    const M3d R2 = BASIC::Exp(V3d(w));
    EXPECT_LT(maxAbsDiff(R, R2), 1e-9);
  }
}

TEST(LogTest, TinyAngleBranch) {
  // BASIC::Exp truncates to identity below 1e-10, so build the first-order
  // rotation matrix directly; theta = 1e-12 drives Log's sin_half < 1e-11
  // small-angle branch (orthonormality error is O(theta^2), far below eps).
  const auto axes = randomUnitAxes(3, 55u);
  for (const auto& axis : axes) {
    const double theta = 1e-12;
    const M3d R = M3d::Identity() + BASIC::SKEW(V3d(axis * theta));
    const V3d w = BASIC::Log(R);
    EXPECT_LT((w - axis * theta).norm(), 1e-15);
  }
  EXPECT_LT(BASIC::Log(M3d(M3d::Identity())).norm(), 1e-15);
}

TEST(LogTest, NormNeverExceedsPi) {
  std::mt19937 rng(77u);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 200; ++i) {
    Eigen::Quaterniond q(u(rng), u(rng), u(rng), u(rng));
    if (q.norm() < 1e-3) continue;
    q.normalize();
    const V3d w = BASIC::Log(M3d(q.toRotationMatrix()));
    EXPECT_LE(w.norm(), M_PI + 1e-9);
    EXPECT_LT(maxAbsDiff(BASIC::Exp(V3d(w)), q.toRotationMatrix()), 1e-9);
  }
}

// ---------------------------------------------------------------------------
// SO3 (scalar = float in this repo, log computed internally in double)
// ---------------------------------------------------------------------------

TEST(SO3Test, ExpLogRoundtrip) {
  const std::vector<double> thetas = {1e-6, 0.1, 1.0, 2.0, 3.0,
                                      M_PI - 1e-4, M_PI - 1e-6, M_PI};
  const auto axes = randomUnitAxes(5, 91u);
  for (const auto& axis : axes) {
    for (double theta : thetas) {
      const V3 phi = (axis * theta).cast<BASIC::scalar>();
      const SO3 R = SO3::Exp(phi);
      const V3 w = R.log_vee();
      // |Log| stays in [0, pi]
      EXPECT_LE(w.norm(), static_cast<BASIC::scalar>(M_PI) + 1e-4f);
      EXPECT_NEAR(w.norm(), std::min(theta, M_PI), 1e-4)
          << "theta=" << theta;
      const SO3 R2 = SO3::Exp(w);
      EXPECT_LT((R.R() - R2.R()).cwiseAbs().maxCoeff(), 3e-5f)
          << "theta=" << theta << " axis=" << axis.transpose();
    }
  }
}

TEST(SO3Test, UpdateIsLeftMultiplication) {
  const auto axes = randomUnitAxes(3, 101u);
  SO3 R = SO3::Exp(V3(0.3f, -0.2f, 0.9f));
  for (const auto& axis : axes) {
    const V3 dw = (axis * 0.05).cast<BASIC::scalar>();
    const M3 expected = SO3::Exp(dw).R() * R.R();
    R.update(dw);
    EXPECT_LT((R.R() - expected).cwiseAbs().maxCoeff(), 1e-6f);
  }
}

TEST(SO3Test, InverseAndDistance) {
  const SO3 R = SO3::Exp(V3(0.4f, 0.5f, -0.6f));
  const M3 should_be_eye = (R * R.inverse()).R();
  EXPECT_LT((should_be_eye - BASIC::Eye3).cwiseAbs().maxCoeff(), 1e-6f);
  EXPECT_NEAR(R.distance(R), 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// SE3
// ---------------------------------------------------------------------------

void expectSE3InternalConsistency(const SE3& T, float tol) {
  // R()/t() must be exactly the blocks of T()
  EXPECT_LE((T.T().topLeftCorner<3, 3>() - T.R()).cwiseAbs().maxCoeff(), tol);
  EXPECT_LE((T.T().topRightCorner<3, 1>() - T.t()).cwiseAbs().maxCoeff(), tol);
  EXPECT_FLOAT_EQ(T.T()(3, 3), 1.0f);
  EXPECT_LE(T.T().row(3).head<3>().cwiseAbs().maxCoeff(), tol);
  // R stays orthonormal
  EXPECT_LT((T.R() * T.R().transpose() - BASIC::Eye3).cwiseAbs().maxCoeff(),
            1e-5f);
}

TEST(SE3Test, UpdateKeepsFieldsConsistent) {
  SE3 T(SO3::Exp(V3(0.2f, -0.4f, 0.7f)).R(), V3(1.0f, -2.0f, 0.5f));
  expectSE3InternalConsistency(T, 0.0f);

  std::mt19937 rng(123u);
  std::uniform_real_distribution<float> u(-0.2f, 0.2f);
  for (int i = 0; i < 20; ++i) {
    V6 dxi;
    dxi << u(rng), u(rng), u(rng), u(rng), u(rng), u(rng);
    const M4 expected = SE3(dxi).T() * T.T();
    T.update(dxi);
    expectSE3InternalConsistency(T, 1e-7f);
    EXPECT_LT((T.T() - expected).cwiseAbs().maxCoeff(), 1e-5f);
    // log/exp of the updated pose must reproduce the same transform
    const SE3 T2(T.log_vee());
    EXPECT_LT((T.T() - T2.T()).cwiseAbs().maxCoeff(), 1e-4f);
  }
}

TEST(SE3Test, UpdateRhsIsRightMultiplication) {
  SE3 T(SO3::Exp(V3(-0.1f, 0.3f, 0.5f)).R(), V3(0.4f, 1.5f, -0.7f));
  V6 dxi;
  dxi << 0.05f, -0.02f, 0.03f, 0.1f, -0.2f, 0.05f;
  const M4 expected = T.T() * SE3(dxi).T();
  T.updateRhs(dxi);
  expectSE3InternalConsistency(T, 1e-7f);
  EXPECT_LT((T.T() - expected).cwiseAbs().maxCoeff(), 1e-5f);
}

TEST(SE3Test, LogIsFiniteAndAccurateForTinyRotations) {
  // Regression test for the SE3::log fix: the closed-form k1 coefficient
  // suffered catastrophic cancellation (wrong values below theta ~ 1e-3 and
  // 0/0 -> inf/NaN below theta ~ 2e-8 where 1-cos underflows to 0).
  const auto axes = randomUnitAxes(3, 219u);
  for (const auto& axis : axes) {
    for (double theta : {1e-9, 1e-7, 1e-5, 1e-4, 1e-3}) {
      V6 xi;
      xi.head<3>() = (axis * theta).cast<BASIC::scalar>();
      xi.tail<3>() = V3(1.5f, -2.0f, 0.5f);
      const SE3 T(xi);
      const V6 xi_back = T.log_vee();
      ASSERT_TRUE(xi_back.allFinite()) << "theta=" << theta;
      EXPECT_LT((xi_back - xi).norm(), 1e-5f * (1.0f + xi.norm()))
          << "theta=" << theta;
    }
  }
}

TEST(SE3Test, ExpLogRoundtrip) {
  const std::vector<double> thetas = {1e-6, 1e-4, 0.1, 1.0, 2.0, 3.0,
                                      M_PI - 1e-4, M_PI - 1e-6};
  const auto axes = randomUnitAxes(4, 131u);
  std::mt19937 rng(17u);
  std::uniform_real_distribution<float> u(-2.0f, 2.0f);
  for (const auto& axis : axes) {
    for (double theta : thetas) {
      V6 xi;
      xi.head<3>() = (axis * theta).cast<BASIC::scalar>();
      xi.tail<3>() = V3(u(rng), u(rng), u(rng));
      const SE3 T(xi);
      const V6 xi_back = T.log_vee();
      EXPECT_LT((xi_back - xi).norm(), 5e-4f * (1.0f + xi.norm()))
          << "theta=" << theta;
      const SE3 T2(xi_back);
      EXPECT_LT((T.T() - T2.T()).cwiseAbs().maxCoeff(), 1e-4f)
          << "theta=" << theta;
    }
  }
}

TEST(SE3Test, InverseComposesToIdentity) {
  const SE3 T(SO3::Exp(V3(0.9f, -0.8f, 0.3f)).R(), V3(3.0f, -1.0f, 2.0f));
  const SE3 I = T * T.inverse();
  EXPECT_LT((I.T() - M4::Identity()).cwiseAbs().maxCoeff(), 1e-5f);
}

}  // namespace
