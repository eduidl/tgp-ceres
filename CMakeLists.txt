cmake_minimum_required(VERSION 3.10)
project(tgp-ceres)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

SET(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

add_library(polyhedron_lib
  ${SRC_DIR}/face.cc
  ${SRC_DIR}/line.cc
  ${SRC_DIR}/point.cc
  ${SRC_DIR}/polyhedron.cc
)
target_compile_features(polyhedron_lib PUBLIC cxx_std_17)
target_compile_options(polyhedron_lib PUBLIC
  -Wall
  -Wextra
  -Wconversion
  -Wpedantic
)
target_include_directories(polyhedron_lib PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(polyhedron_lib PRIVATE
  Eigen3::Eigen  
)

add_executable(main
  ${SRC_DIR}/main.cc
)
target_compile_features(main PUBLIC cxx_std_17)
target_compile_options(main PUBLIC
  -Wall
  -Wextra
  -Wconversion
  -Wpedantic
)
target_include_directories(main PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${CERES_INCLUDE_DIR}
)
target_link_libraries(main PRIVATE
  ${CERES_LIBRARIES}
  polyhedron_lib
)
