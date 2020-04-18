#pragma once

#include <array>
#include <cassert>
#include <vector>

#include <Eigen/Dense>

#include "line.h"
#include "point.h"

namespace polyhedron {

template <typename T>
class Square {
 public:
  Square(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3, PointPtr<T> p4) : points_({p1, p2, p3, p4}) {
    assert(p1 != nullptr && p2 != nullptr && p3 != nullptr && p4 != nullptr);
  }
  virtual ~Square() = default;

  auto Edges() const {
    return std::vector<LineSegment<T>>{
        {points_[0], points_[1]}, {points_[1], points_[2]}, {points_[2], points_[3]}, {points_[3], points_[0]}};
  };

  auto Diagonals() const { return std::vector<LineSegment<T>>{{points_[0], points_[2]}, {points_[1], points_[3]}}; }

 private:
  const std::array<PointPtr<T>, 4> points_;
};

template <typename T>
class Triangle {
 public:
  Triangle(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3) : points_({p1, p2, p3}) {
    assert(p1 != nullptr && p2 != nullptr && p3 != nullptr);
  }
  virtual ~Triangle() = default;

  auto Edges() const {
    return std::vector<LineSegment<T>>{{points_[0], points_[1]}, {points_[1], points_[2]}, {points_[2], points_[0]}};
  }

 private:
  const std::array<PointPtr<T>, 3> points_;
};

}  // namespace polyhedron
