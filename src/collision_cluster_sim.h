#pragma once

#include "collision_cluster_config.h"
#include "spinning_particle_engine.h"

class CollisionClusterSim
    : public SpinningParticleEngine<CollisionClusterSim> {
 public:
  explicit CollisionClusterSim(const CollisionClusterConfig& cfg);

  void ApplyForces();

 private:
  CollisionClusterConfig cfg_;
};
