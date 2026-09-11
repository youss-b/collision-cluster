#include <string>

#include "collision_cluster_sim.h"
#include "config_loader.h"
#include "file_io.h"

int main(int argc, char* argv[]) {
  std::string config_path = (argc > 1) ? argv[1] : "config.cfg";
  CollisionClusterConfig cfg = LoadConfig(config_path);

  CollisionClusterSim sim(cfg);
  sim.Run();

  if (cfg.write_to_file) {
    FileIO::WriteFullSim(cfg.output_dir, sim.GetRecorded(),
                         sim.num_extra_params());
  }

  return 0;
}
