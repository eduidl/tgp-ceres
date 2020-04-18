cmake_minimum_required(VERSION 3.10)
project(tgp-ceres)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(polyhedron INTERFACE)
target_compile_features(polyhedron INTERFACE cxx_std_17)
target_include_directories(polyhedron INTERFACE
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(polyhedron INTERFACE
  Eigen3::Eigen  
)

add_executable(main
  ${PROJECT_SOURCE_DIR}/main.cc
)
target_compile_features(main PRIVATE cxx_std_17)
target_compile_options(main PRIVATE
  -Wall
  -Wextra
  -Wconversion
  -Wpedantic
)
target_include_directories(main PRIVATE
  ${CERES_INCLUDE_DIR}
)
target_link_libraries(main PRIVATE
  ${CERES_LIBRARIES}
  polyhedron
)
