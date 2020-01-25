#pragma once

#include <array>
#include <vector>

#include "line.h"

namespace app {

template <typename T>
class Point;

template <typename T>
using PointPtr = std::shared_ptr<Point<T>>;

template <typename T>
class Square {
 public:
  Square(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3, PointPtr<T> p4);
  virtual ~Square() = default;

  std::vector<LineSegment<T>> Edges() const;
  std::vector<LineSegment<T>> Diagonals() const;

 private:
  const std::array<PointPtr<T>, 4> points_;
};

template <typename T>
class Triangle {
 public:
  Triangle(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3);
  ~Triangle() = default;

  std::vector<LineSegment<T>> Edges() const;

 private:
  const std::array<PointPtr<T>, 3> points_;
};

}  // namespace app
