#pragma once

#include <cmath>
#include <memory>

#include <ceres/ceres.h>

#include "polyhedron.h"

namespace app {

struct PolyhedronCostFunctor {
  explicit PolyhedronCostFunctor(std::shared_ptr<Polyhedron<double>> polyhedron) : polyhedron_(polyhedron) {}
  virtual ~PolyhedronCostFunctor() = default;

  bool operator()(double const* const* parameters, double* residual) const {
    polyhedron_->UpdateParams(parameters[0], parameters[1]);
    auto* cur_res = residual;
    for (auto it = polyhedron_->Edges().cbegin(); it < polyhedron_->Edges().cend(); ++it, ++cur_res) {
      *cur_res = it->Length() - polyhedron_->EdgeLengthMean();
    }
    for (auto it = polyhedron_->Diagonals().cbegin(); it < polyhedron_->Diagonals().cend(); ++it, ++cur_res) {
      *cur_res = kBeta * (it->Length() / std::sqrt(2.) - polyhedron_->EdgeLengthMean());
    }
    return true;
  }

  static auto Create(std::shared_ptr<Polyhedron<double>> polyhedron) {
    const auto abs_point_size = static_cast<int>(polyhedron->AbsPointSize());
    const auto residual_size = polyhedron->Edges().size() + polyhedron->Diagonals().size();

    auto cost_function =
        new ceres::DynamicNumericDiffCostFunction<PolyhedronCostFunctor>(new PolyhedronCostFunctor(polyhedron));
    cost_function->AddParameterBlock(abs_point_size);
    cost_function->AddParameterBlock(abs_point_size);
    cost_function->SetNumResiduals(static_cast<int>(residual_size));

    return cost_function;
  }

 private:
  // weight of cost to diagonal lenght
  static constexpr double kBeta = 0.7;

  std::shared_ptr<Polyhedron<double>> polyhedron_;
};

}  // namespace app
