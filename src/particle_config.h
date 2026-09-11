#pragma once

#include <string>

struct ParticleBaseConfig {
  int dims = 3;
  int universe_size = 10;

  float friction = 0.0f;

  // Per-particle radii are sampled uniformly from [min_radius, max_radius].
  // Single-radius sims set them equal.
  float min_radius = 0.1f;
  float max_radius = 0.1f;

  enum class InitStrategy { Spaced, Random };
  InitStrategy init_strategy = InitStrategy::Spaced;

  int framerate = 60;
  int seconds = 20;
  int substeps = 8;

  bool write_to_file = true;
  std::string output_dir = "output";

  // Derived — set by FinalizeBaseConfig(), not by user
  int num_particles = 0;
  float dt = 0.0f;
  float sub_dt = 0.0f;
};

inline void FinalizeBaseConfig(ParticleBaseConfig& cfg) {
  if (cfg.init_strategy == ParticleBaseConfig::InitStrategy::Spaced) {
    int n = cfg.universe_size;
    cfg.num_particles = (cfg.dims == 2) ? n * n : n * n * n;
  }
  // For Random, caller sets num_particles before calling
  // FinalizeBaseConfig.

  cfg.dt = 1.0f / cfg.framerate;
  cfg.sub_dt = cfg.dt / cfg.substeps;
}
