#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "polyhedron.h"
#include "problem.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  if (argc < 3) {
    std::cout << "Insufficient number of arguments. Please exetute `./main <h> <k>`" << std::endl;
    return -1;
  }

  const auto h = std::max(std::atoi(argv[1]), std::atoi(argv[2]));
  const auto k = std::min(std::atoi(argv[1]), std::atoi(argv[2]));

  if (h < 0 || k < 0) {
    std::cout << "Both h and k should be not less than zero." << std::endl;
    return -1;
  } else if (h == 0 && k == 0) {
    std::cout << "Either h or k should be not zero." << std::endl;
    return -1;
  }

  auto polyhedron = std::make_shared<app::Polyhedron<double>>(h, k);
  const auto abs_point_size = polyhedron->AbsPointSize();

  std::vector<double> theta_vec(abs_point_size);
  std::vector<double> phi_vec(abs_point_size);
  polyhedron->GetInitialParams(theta_vec, phi_vec);

  ceres::Problem problem;
  auto cost = app::PolyhedronCostFunctor::Create(polyhedron);
  problem.AddResidualBlock(cost, NULL, theta_vec.data(), phi_vec.data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::string out_fname = "../visualize/data.ts";
  polyhedron->DumpTs(out_fname);
}
