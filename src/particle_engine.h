#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "particle_config.h"

template <typename Derived>
class ParticleEngine {
 public:
  explicit ParticleEngine(const ParticleBaseConfig& cfg)
      : base_cfg_(cfg),
        damping_(std::exp(-cfg.friction * cfg.sub_dt)),
        sub_dt_sq_(cfg.sub_dt * cfg.sub_dt) {
    const int N = base_cfg_.num_particles;
    pos_.resize(3, N);
    old_pos_.resize(3, N);
    accel_.resize(3, N);

    const float r_min = base_cfg_.min_radius;
    const float r_span = base_cfg_.max_radius - base_cfg_.min_radius;
    radius_ = (Eigen::ArrayXf::Random(N) + 1.0f) * (0.5f * r_span) + r_min;
    inv_mass_ = radius_.array().cube().inverse();
  }

  void Run() {
    auto start = std::chrono::steady_clock::now();

    int total_frames = base_cfg_.seconds * base_cfg_.framerate;
    int vals_per_particle = 3 + derived().num_extra_params();
    recorded_.resize(
        total_frames,
        std::vector<std::vector<float>>(base_cfg_.num_particles,
                                        std::vector<float>(vals_per_particle)));

    for (int frame = 0; frame < total_frames; ++frame) {
      for (int step = 0; step < base_cfg_.substeps; ++step) {
        StepSubstep();
      }
      recorded_[frame] = derived().CaptureFrame();
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSim took "
              << std::chrono::duration<double>(end - start).count()
              << " seconds.\n\n";
  }

  const std::vector<std::vector<std::vector<float>>>& GetRecorded() const {
    return recorded_;
  }

  // Default capture: [x, y, z, r] per particle. Override in Derived for extras.
  std::vector<std::vector<float>> CaptureFrame() const {
    int N = pos_.cols();
    std::vector<std::vector<float>> result(N, std::vector<float>(4));
    for (int i = 0; i < N; ++i) {
      result[i][0] = pos_(0, i);
      result[i][1] = pos_(1, i);
      result[i][2] = pos_(2, i);
      result[i][3] = radius_(i);
    }
    return result;
  }

  int num_extra_params() const { return 1; }

  // Default hook called by StepSubstep after Derived::ApplyForces() and before
  // the verlet position update. SpinningParticleEngine overrides this to run
  // angular friction + orientation integration; non-spinning sims inherit the
  // no-op default.
  void PostForces() {}

 protected:
  void InitRandom() {
    pos_ = Eigen::Matrix3Xf::Random(3, base_cfg_.num_particles) *
           float(base_cfg_.universe_size);
    if (base_cfg_.dims == 2) pos_.row(2).setZero();
  }

  void InitSpaced() {
    int n = base_cfg_.universe_size;
    Eigen::VectorXf init_pos =
        Eigen::VectorXf::LinSpaced(n, 0, n).array() - (n * 0.5f);

    pos_.row(0) = init_pos.replicate(1, (base_cfg_.dims == 3) ? n * n : n)
                      .transpose()
                      .reshaped()
                      .transpose();
    if (base_cfg_.dims == 3) {
      pos_.row(1) =
          init_pos.replicate(1, n)
              .transpose()
              .reshaped()
              .transpose()
              .replicate(1, n);
    } else {
      pos_.row(1) = init_pos.transpose().replicate(1, n);
    }
    if (base_cfg_.dims == 3) {
      pos_.row(2) = init_pos.transpose().replicate(1, n * n);
    } else {
      pos_.row(2).setZero();
    }
  }

  ParticleBaseConfig base_cfg_;
  const float       damping_;
  const float       sub_dt_sq_;
  Eigen::Matrix3Xf  pos_;
  Eigen::Matrix3Xf  old_pos_;
  Eigen::Matrix3Xf  accel_;
  Eigen::VectorXf   radius_;
  Eigen::VectorXf   inv_mass_;
  std::vector<std::vector<std::vector<float>>> recorded_;

 private:
  void StepSubstep() {
    accel_.setZero();
    derived().ApplyForces();
    derived().PostForces();

    Eigen::Matrix3Xf tmp = pos_;
    pos_ = pos_ + pos_ * damping_ - old_pos_ * damping_ + accel_ * sub_dt_sq_;
    old_pos_ = tmp;
  }

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }
};
