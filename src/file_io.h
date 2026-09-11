#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace FileIO {

inline const std::chrono::steady_clock::time_point StartClock() {
  return std::chrono::steady_clock::now();
}

inline void EndClock(const std::chrono::steady_clock::time_point& start,
              const std::filesystem::path& output_dir) {
  const auto end = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = end - start;
  std::cout << "\nFile processing took " << elapsed.count() << " seconds\n "
            << " and wrote to " << std::filesystem::absolute(output_dir)
            << std::endl;
}

inline std::filesystem::path GetDirectory(const std::string output_directory_name) {
  std::filesystem::path output_dir{output_directory_name};
  std::filesystem::create_directories(output_dir);
  // Clear out the previous run, but leave dotfiles alone so the directory's
  // .gitkeep survives (a plain remove_all on the directory deletes it).
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    if (entry.path().filename().string().starts_with(".")) continue;
    std::filesystem::remove_all(entry.path());
  }
  return output_dir;
}

inline void WriteFullSim(const std::string output_directory_name,
                  const std::vector<std::vector<std::vector<float>>>& positions,
                  const int num_extra_params = 0) {
  const auto start = StartClock();
  auto output_dir = GetDirectory(output_directory_name);

  std::stringstream ss;

  for (int i = 0; i < positions.size(); ++i) {
    ss.str(std::string());
    ss << "sim_" << std::setw(4) << std::setfill('0') << i << ".csv";
    std::string filename = ss.str();
    std::ofstream frame_file;
    frame_file.open(output_dir / filename);
    if (!(i % 100)) {
      std::cout << '\r' << "Processing frame " << i << " of "
              << positions.size() << std::flush;
    }
    if (!frame_file) {
      std::cout << "Failed to write to file " << filename << std::endl;
      return;
    }
    frame_file << "x,y,z,";
    for (int param = 0; param < num_extra_params; ++param) {
      frame_file << (param + 1) << ',';
    }
    frame_file << "\n";
    for (int j = 0; j < positions[i].size(); ++j) {
      for (int k = 0; k < positions[i][j].size(); ++k) {
        frame_file << positions[i][j][k] << ",";
      }
      frame_file << std::endl;
    }
    frame_file.close();
  }
  EndClock(start, output_dir);
}

}  // namespace FileIO
