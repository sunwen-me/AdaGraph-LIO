// Unit tests for AdaGraphBackend covariance logic, observed through the
// public processKeyframe() result (result.sigmas = sqrt(diag(cov))):
//  - strong observations take the Hessian-inverse path
//  - directionally degenerate Hessians blend: observed directions use 1/lambda,
//    degenerate directions get the rule-based variance injected
//  - empty hessian_terms fall back to the pure rule-based covariance
//  - an all-weak Hessian (max eigenvalue below the gate) also falls back
//
// Tangent ordering is [rot(0..2), trans(3..5)]; backend constants (from
// adagraph_backend.h): base_sigma_t = 0.035, base_sigma_r = 0.012,
// hessian_eig_min = 1e3.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "lio/adagraph_backend.h"
#include "lio/adagraph_types.h"

namespace {

class LoggingEnvironment : public ::testing::Environment {
 public:
  void SetUp() override { BASIC::init_log(); }
};

const auto* const logging_environment =
    ::testing::AddGlobalTestEnvironment(new LoggingEnvironment());

using LI2Sup::AdaGraphBackend;
using LI2Sup::BackendResult;
using LI2Sup::FeatureFrame;
using BASIC::M6;
using BASIC::SE3;
using BASIC::SO3;
using BASIC::V3;
using BASIC::V6;

constexpr double kBaseSigmaT = 0.035;
constexpr double kBaseSigmaR = 0.012;

using HessianTerms = std::vector<std::pair<M6, V6>>;

// A frame that triggers none of the rule-based inflation tiers (scale 1.0).
FeatureFrame goodFrame() {
  FeatureFrame f;
  f.inlier_ratio = 0.9;
  f.condition_number = 100.0;
  f.hessian_trace = 1e7;
  f.effective_point_number = 5000;
  f.angular_velocity_norm = 0.1;
  return f;
}

BackendResult process(AdaGraphBackend& backend, const FeatureFrame& frame,
                      const HessianTerms& terms, double timestamp = 0.0,
                      const SE3& pose = SE3()) {
  return backend.processKeyframe(frame, pose, terms, timestamp);
}

TEST(AdaGraphBackendCovariance, EmptyTermsUseRuleBasedCovarianceGoodFrame) {
  AdaGraphBackend backend;
  backend.initWithConfig();
  ASSERT_TRUE(backend.initialized());

  const BackendResult res = process(backend, goodFrame(), {});
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaR, 1e-9) << "rot index " << i;
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaT, 1e-9) << "trans index " << i;
  }
}

TEST(AdaGraphBackendConfig, ExplicitInstancesDoNotShareGlobalTuning) {
  adagraph::backend::BackendConfig precise_config;
  precise_config.odometry_translation_sigma_m = 0.02;
  precise_config.odometry_rotation_sigma_rad = 0.01;
  AdaGraphBackend precise_backend(precise_config);
  precise_backend.initWithConfig();

  adagraph::backend::BackendConfig loose_config;
  loose_config.odometry_translation_sigma_m = 0.2;
  loose_config.odometry_rotation_sigma_rad = 0.1;
  AdaGraphBackend loose_backend(loose_config);
  loose_backend.initWithConfig();

  const BackendResult precise = process(precise_backend, goodFrame(), {});
  const BackendResult loose = process(loose_backend, goodFrame(), {});
  EXPECT_NEAR(precise.sigmas(0), 0.01, 1e-9);
  EXPECT_NEAR(precise.sigmas(3), 0.02, 1e-9);
  EXPECT_NEAR(loose.sigmas(0), 0.1, 1e-9);
  EXPECT_NEAR(loose.sigmas(3), 0.2, 1e-9);
}

TEST(AdaGraphBackendConfig, RejectsInvalidExplicitConfig) {
  adagraph::backend::BackendConfig config;
  config.keyframe_max_frames = 0;
  AdaGraphBackend backend(config);
  EXPECT_THROW(backend.initWithConfig(), std::invalid_argument);
}

TEST(AdaGraphBackendCovariance, RuleBasedTierModerateInlierRatio) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  FeatureFrame f = goodFrame();
  f.inlier_ratio = 0.6;  // 0.5..0.7 tier -> x1.3
  const BackendResult res = process(backend, f, {});
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaR * 1.3, 1e-9);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaT * 1.3, 1e-9);
  }
}

TEST(AdaGraphBackendCovariance, RuleBasedScalesClampAtTen) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  FeatureFrame f;
  f.inlier_ratio = 0.1;             // x3 (both)
  f.condition_number = 1e5;         // x3 (both)
  f.hessian_trace = 1e4;            // x1.5 (both)
  f.effective_point_number = 100;   // x1.5 (both)
  f.angular_velocity_norm = 2.0;    // rot x2, trans x1.5
  // raw: s_t = 3*3*1.5*1.5*1.5 = 30.375, s_r = 3*3*1.5*1.5*2 = 40.5
  // -> both clamp to 10.
  const BackendResult res = process(backend, f, {});
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaR * 10.0, 1e-9);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaT * 10.0, 1e-9);
  }
}

TEST(AdaGraphBackendCovariance, StrongHessianUsesInformationInverse) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  // Two terms sum to H = 4e5 * I (information is additive), all eigenvalues
  // far above the 1e3 gate -> cov = H^-1, sigmas = 1/sqrt(4e5).
  HessianTerms terms;
  terms.emplace_back(M6(M6::Identity() * 2e5f), V6::Zero());
  terms.emplace_back(M6(M6::Identity() * 2e5f), V6::Zero());

  const BackendResult res = process(backend, goodFrame(), terms);
  const double expected = 1.0 / std::sqrt(4e5);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), expected, expected * 1e-4) << "index " << i;
  }
}

TEST(AdaGraphBackendCovariance, DegenerateAxisGetsRuleVarianceInjected) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  // rot-z (index 2) nearly unobserved (eigenvalue 10 < gate 1e3), all other
  // directions strong (1e6). Expect sigma(2) = rule sigma_r, others 1e-3.
  M6 H = M6::Identity() * 1e6f;
  H(2, 2) = 10.0f;
  HessianTerms terms;
  terms.emplace_back(H, V6::Zero());

  const BackendResult res = process(backend, goodFrame(), terms);
  const double strong = 1e-3;  // 1/sqrt(1e6)
  EXPECT_NEAR(res.sigmas(0), strong, strong * 1e-4);
  EXPECT_NEAR(res.sigmas(1), strong, strong * 1e-4);
  EXPECT_NEAR(res.sigmas(2), kBaseSigmaR, kBaseSigmaR * 1e-4);
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), strong, strong * 1e-4) << "index " << i;
  }
}

TEST(AdaGraphBackendCovariance, DegenerateTranslationAxisUsesRuleSigmaT) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  // trans-x (index 3) degenerate -> rule sigma_t there.
  M6 H = M6::Identity() * 1e6f;
  H(3, 3) = 1.0f;
  HessianTerms terms;
  terms.emplace_back(H, V6::Zero());

  const BackendResult res = process(backend, goodFrame(), terms);
  EXPECT_NEAR(res.sigmas(3), kBaseSigmaT, kBaseSigmaT * 1e-4);
  const double strong = 1e-3;
  for (int i : {0, 1, 2, 4, 5}) {
    EXPECT_NEAR(res.sigmas(i), strong, strong * 1e-4) << "index " << i;
  }
}

TEST(AdaGraphBackendCovariance, DegenerateOffAxisDirectionBlends) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  // Degenerate direction v = (1,1,0,0,0,0)/sqrt(2) in the rotation block:
  // H = 1e6 * (I - v v^T) + 10 * v v^T. The blended covariance is
  // 1e-6 * (I - v v^T) + sigma_r^2 * v v^T, so
  // cov(0,0) = cov(1,1) = 0.5e-6 + 0.5*sigma_r^2.
  Eigen::Matrix<float, 6, 1> v = Eigen::Matrix<float, 6, 1>::Zero();
  v(0) = v(1) = static_cast<float>(1.0 / std::sqrt(2.0));
  const M6 P = v * v.transpose();
  const M6 H = 1e6f * (M6::Identity() - P) + 10.0f * P;
  HessianTerms terms;
  terms.emplace_back(H, V6::Zero());

  const BackendResult res = process(backend, goodFrame(), terms);
  const double mixed =
      std::sqrt(0.5 * 1e-6 + 0.5 * kBaseSigmaR * kBaseSigmaR);
  EXPECT_NEAR(res.sigmas(0), mixed, mixed * 1e-3);
  EXPECT_NEAR(res.sigmas(1), mixed, mixed * 1e-3);
  const double strong = 1e-3;
  EXPECT_NEAR(res.sigmas(2), strong, strong * 1e-3);
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), strong, strong * 1e-3) << "index " << i;
  }
}

TEST(AdaGraphBackendCovariance, AllWeakHessianFallsBackToRulePath) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  // max eigenvalue 100 < hessian_eig_min (1e3) -> whole Hessian rejected,
  // pure rule-based covariance.
  HessianTerms terms;
  terms.emplace_back(M6(M6::Identity() * 100.0f), V6::Zero());

  const BackendResult res = process(backend, goodFrame(), terms);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaR, 1e-9);
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(res.sigmas(i), kBaseSigmaT, 1e-9);
  }
}

TEST(AdaGraphBackendChain, PureOdometryChainReproducesInputPoses) {
  AdaGraphBackend backend;
  backend.initWithConfig();

  const SE3 p0;  // identity
  const SE3 p1(SO3::Exp(V3(0.0f, 0.0f, 0.1f)).R(), V3(1.0f, 0.0f, 0.0f));

  const BackendResult r0 = process(backend, goodFrame(), {}, 0.0, p0);
  EXPECT_EQ(r0.keyframe_count, 1u);
  EXPECT_FALSE(r0.optimized);  // no localization factors -> pure chain
  EXPECT_LT((r0.optimized_pose.t() - p0.t()).norm(), 1e-4f);

  const BackendResult r1 = process(backend, goodFrame(), {}, 0.1, p1);
  EXPECT_EQ(r1.keyframe_count, 2u);
  EXPECT_LT((r1.optimized_pose.t() - p1.t()).norm(), 1e-4f);
  EXPECT_LT((r1.optimized_pose.R() - p1.R()).cwiseAbs().maxCoeff(), 1e-4f);
}

}  // namespace
