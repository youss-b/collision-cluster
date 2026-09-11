#pragma once

#include "particle_config.h"

struct SpinningParticleBaseConfig : ParticleBaseConfig {
  // Bound on per-component initial angular velocity (rad/s).
  float max_angular_speed = 2.0f;

  // Per-substep blend rate of perpendicular angular velocity at a contact
  // (the contact-tangent spin blend).
  float collision_spin_transfer = 0.05f;

  // Per-substep multiplicative damping on angular velocity. Auto-applied by
  // SpinningParticleEngine once per substep, before orientation integration.
  float angular_friction = 0.0f;
};
