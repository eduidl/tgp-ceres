#pragma once

#include <memory>

#include <ceres/ceres.h>

#include "polyhedron.h"

namespace app {

struct PolyhedronCostFunctor {
  explicit PolyhedronCostFunctor(std::shared_ptr<Polyhedron<double>> polyhedron) : polyhedron_(polyhedron) {}
  virtual ~PolyhedronCostFunctor() = default;

  bool operator()(double const* const* parameters, double* residual) const {
    polyhedron_->UpdateParams(parameters[0], parameters[1]);
    size_t residual_i = 0;
    for (size_t i = 0; i < polyhedron_->EdgeSize(); ++i, ++residual_i) {
      residual[residual_i] = polyhedron_->EdgeLength(i) - polyhedron_->EdgeLengthMean();
    }
    for (size_t i = 0; i < polyhedron_->DiagonalSize(); ++i, ++residual_i) {
      residual[residual_i] = 0.8 * (polyhedron_->DiagonalLength(i) / std::sqrt(2.) - polyhedron_->EdgeLengthMean());
    }
    return true;
  }

  static auto Create(std::shared_ptr<Polyhedron<double>> polyhedron) {
    const auto abs_point_size = static_cast<int>(polyhedron->AbsPointSize());
    const auto residual_size = static_cast<int>(polyhedron->EdgeSize() + polyhedron->DiagonalSize());

    auto cost_function =
        new ceres::DynamicNumericDiffCostFunction<PolyhedronCostFunctor>(new PolyhedronCostFunctor(polyhedron));
    cost_function->AddParameterBlock(abs_point_size);
    cost_function->AddParameterBlock(abs_point_size);
    cost_function->SetNumResiduals(residual_size);

    return cost_function;
  }

  std::shared_ptr<Polyhedron<double>> polyhedron_;
};

}  // namespace app
