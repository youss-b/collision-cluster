#include "collision_cluster_sim.h"

#include "particle_collisions.h"

CollisionClusterSim::CollisionClusterSim(const CollisionClusterConfig& cfg)
    : SpinningParticleEngine(cfg), cfg_(cfg) {
  if (cfg_.init_strategy == ParticleBaseConfig::InitStrategy::Spaced) {
    InitSpaced();
  } else {
    InitRandom();
  }
  old_pos_ = pos_;
}

void CollisionClusterSim::ApplyForces() {
  if (cfg_.gravity_to_origin != 0.0f) {
    Eigen::Matrix3Xf dir = -pos_;
    Eigen::RowVectorXf norms = dir.colwise().norm();
    norms = (norms.array() < 1e-6f).select(1.0f, norms);
    dir.array().rowwise() /= norms.array();
    accel_ += dir * cfg_.gravity_to_origin;
  }

  ApplyPairwiseCollisionsWithSpin(pos_, cfg_.num_particles, radius_, inv_mass_,
                                  angular_vel_, cfg_.collision_spin_transfer);
}
