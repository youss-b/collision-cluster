#pragma once

#include "spinning_particle_config.h"

struct CollisionClusterConfig : SpinningParticleBaseConfig {
  // Acceleration magnitude pulling each particle toward the origin.
  float gravity_to_origin = 5.0f;
};

inline CollisionClusterConfig MakeDefaultConfig() {
  CollisionClusterConfig cfg;
  cfg.init_strategy = ParticleBaseConfig::InitStrategy::Random;
  cfg.num_particles = 500;
  return cfg;
}

inline void FinalizeConfig(CollisionClusterConfig& cfg) {
  FinalizeBaseConfig(cfg);
}
