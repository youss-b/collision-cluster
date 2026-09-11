cmake_minimum_required(VERSION 3.28)
project(CollisionCluster LANGUAGES CXX)

# Set compiler flags here. -O3 is best to optimize Eigen.
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} src/main.cc src/collision_cluster_sim.cc)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
