#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "particle_engine.h"
#include "spinning_particle_config.h"

template <typename Derived>
class SpinningParticleEngine : public ParticleEngine<Derived> {
 public:
  explicit SpinningParticleEngine(const SpinningParticleBaseConfig& cfg)
      : ParticleEngine<Derived>(cfg), spin_cfg_(cfg) {
    const int N = cfg.num_particles;

    orientation_.resize(4, N);
    orientation_.row(0).setOnes();
    orientation_.bottomRows<3>().setZero();

    angular_vel_ =
        Eigen::Matrix3Xf::Random(3, N) * cfg.max_angular_speed;

    dq_scratch_.resize(4, N);
  }

  // Default capture: [x, y, z, r, qw, qx, qy, qz]. Override in Derived for
  // additional project-specific columns; canonical engine columns must come
  // first in any override.
  std::vector<std::vector<float>> CaptureFrame() const {
    const int N = this->pos_.cols();
    std::vector<std::vector<float>> result(N, std::vector<float>(8));
    for (int i = 0; i < N; ++i) {
      result[i][0] = this->pos_(0, i);
      result[i][1] = this->pos_(1, i);
      result[i][2] = this->pos_(2, i);
      result[i][3] = this->radius_(i);
      result[i][4] = orientation_(0, i);
      result[i][5] = orientation_(1, i);
      result[i][6] = orientation_(2, i);
      result[i][7] = orientation_(3, i);
    }
    return result;
  }

  int num_extra_params() const { return 5; }

  // Auto-applied by base StepSubstep after Derived::ApplyForces() and before
  // the verlet position update. Runs angular friction then integrates
  // orientation from angular velocity. CRTP picks this up automatically as
  // long as Derived doesn't define its own PostForces().
  void PostForces() {
    if (spin_cfg_.angular_friction != 0.0f) {
      angular_vel_ *= (1.0f - spin_cfg_.angular_friction);
    }
    IntegrateOrientations();
  }

 protected:
  SpinningParticleBaseConfig spin_cfg_;
  Eigen::Matrix4Xf orientation_;  // rows: w, x, y, z
  Eigen::Matrix3Xf angular_vel_;

 private:
  // Vectorized over particles. dq depends on the old orientation across all
  // four components, so we compute the full dq into a scratch matrix first,
  // then step and renormalize column-wise.
  void IntegrateOrientations() {
    const float dt = this->base_cfg_.sub_dt;

    const auto W  = orientation_.row(0).array();
    const auto X  = orientation_.row(1).array();
    const auto Y  = orientation_.row(2).array();
    const auto Z  = orientation_.row(3).array();
    const auto OX = angular_vel_.row(0).array();
    const auto OY = angular_vel_.row(1).array();
    const auto OZ = angular_vel_.row(2).array();

    // dq/dt = 0.5 * (0, ω) * q
    dq_scratch_.row(0).array() = -0.5f * (OX * X + OY * Y + OZ * Z);
    dq_scratch_.row(1).array() =  0.5f * (OX * W + OY * Z - OZ * Y);
    dq_scratch_.row(2).array() =  0.5f * (-OX * Z + OY * W + OZ * X);
    dq_scratch_.row(3).array() =  0.5f * (OX * Y - OY * X + OZ * W);

    orientation_ += dt * dq_scratch_;

    Eigen::RowVectorXf inv_norms =
        orientation_.colwise().norm().cwiseInverse();
    orientation_.array().rowwise() *= inv_norms.array();
  }

  Eigen::Matrix4Xf dq_scratch_;
};
