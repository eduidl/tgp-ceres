#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <tuple>

#include <Eigen/Dense>

#include "eigen_alias.h"

namespace app {

template <typename T>
class Point;

template <typename T>
using PointPtr = std::shared_ptr<Point<T>>;

template <typename T>
class Point {
 public:
  Point(size_t id, uint x, uint y, uint f, uint d);
  virtual ~Point() = default;

  std::tuple<T, T> GetParams() const;
  void SetParams(T theta, T phi);

  size_t Id() const noexcept;
  Vector<T, 3> ToVector() const;

  bool IsAbsolute() const;
  void SetRelativePoint(PointPtr<T> relative_point, Matrix3<T> conversion_mat);
  T DistanceTo(const Point<T> &other) const;

  /**
   * 相対位置の連なりを一つにまとめる
   */
  void ShortenRelativePath();

 private:
  bool RelativePathIsShortened() const;

  size_t id_;
  T theta_;
  T phi_;
  PointPtr<T> relative_point_;
  std::optional<Matrix3<T>> conversion_mat_;
};

}  // namespace app
