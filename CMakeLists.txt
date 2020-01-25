cmake_minimum_required(VERSION 3.0)
project(ceres)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB SRC ${PROJECT_SOURCE_DIR}/src/*.cc)

add_executable(main ${SRC})
target_compile_features(main PUBLIC cxx_std_17)
target_compile_options(main PUBLIC
  -Wall
  -Wextra
  -Wconversion
  -Wpedantic
)
target_include_directories(main PUBLIC
  ${CERES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(main PUBLIC
  ${CERES_LIBRARIES}
  Eigen3::Eigen  
)
