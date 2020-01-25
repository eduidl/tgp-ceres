#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "polyhedron.h"
#include "problem.h"

int main(int, char** argv) {
  google::InitGoogleLogging(argv[0]);

  const auto h = std::max(std::atoi(argv[1]), std::atoi(argv[2]));
  const auto k = std::min(std::atoi(argv[1]), std::atoi(argv[2]));
  auto poly = std::make_shared<app::Polyhedron<double>>(h, k);
  const auto abs_point_size = poly->AbsPointSize();

  std::vector<double> theta_vec(abs_point_size);
  std::vector<double> phi_vec(abs_point_size);
  poly->GetInitialParams(theta_vec, phi_vec);

  ceres::Problem problem;
  auto cost = app::PolyhedronCostFunctor::Create(poly);
  problem.AddResidualBlock(cost, NULL, theta_vec.data(), phi_vec.data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::string out_fname = "../visualize/data.ts";
  poly->DumpTs(out_fname);
}
