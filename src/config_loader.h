#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "collision_cluster_config.h"

inline std::string Trim(const std::string& s) {
  size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  size_t end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

inline CollisionClusterConfig LoadConfig(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open())
    throw std::runtime_error("Cannot open config file: " + path);

  CollisionClusterConfig cfg = MakeDefaultConfig();

  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(file, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') continue;
    auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    kv[Trim(line.substr(0, eq))] = Trim(line.substr(eq + 1));
  }

  auto get_int = [&](const std::string& k, int& out) {
    if (kv.count(k)) out = std::stoi(kv[k]);
  };
  auto get_float = [&](const std::string& k, float& out) {
    if (kv.count(k)) out = std::stof(kv[k]);
  };
  auto require_float = [&](const std::string& k, float& out) {
    if (!kv.count(k))
      throw std::runtime_error("Required config key missing: " + k);
    out = std::stof(kv[k]);
  };
  auto get_bool = [&](const std::string& k, bool& out) {
    if (kv.count(k)) out = (kv[k] == "true" || kv[k] == "1");
  };
  auto get_str = [&](const std::string& k, std::string& out) {
    if (kv.count(k)) out = kv[k];
  };

  get_int("dims", cfg.dims);
  get_int("universe_size", cfg.universe_size);
  get_int("num_particles", cfg.num_particles);
  get_float("friction", cfg.friction);
  get_int("framerate", cfg.framerate);
  get_int("seconds", cfg.seconds);
  get_int("substeps", cfg.substeps);
  get_bool("write_to_file", cfg.write_to_file);
  get_str("output_dir", cfg.output_dir);

  get_float("gravity_to_origin", cfg.gravity_to_origin);
  require_float("min_radius", cfg.min_radius);
  require_float("max_radius", cfg.max_radius);
  get_float("max_angular_speed", cfg.max_angular_speed);
  get_float("collision_spin_transfer", cfg.collision_spin_transfer);
  get_float("angular_friction", cfg.angular_friction);

  if (kv.count("init_strategy")) {
    std::string val = kv["init_strategy"];
    if (val == "Random" || val == "random") {
      cfg.init_strategy = ParticleBaseConfig::InitStrategy::Random;
    } else {
      cfg.init_strategy = ParticleBaseConfig::InitStrategy::Spaced;
    }
  }

  FinalizeConfig(cfg);
  return cfg;
}
