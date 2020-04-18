#pragma once

#include <cassert>
#include <cmath>
#include <memory>
#include <optional>
#include <tuple>

#include <Eigen/Dense>

#include "eigen_alias.h"

namespace polyhedron {

template <typename T>
class Point;

template <typename T>
using PointPtr = std::shared_ptr<Point<T>>;

template <typename T>
class Point {
 public:
  Point(size_t id, uint x, uint y, uint f, uint d);
  virtual ~Point() = default;

  auto GetParams() const { return std::make_tuple(theta_, phi_); }

  void SetParams(T theta, T phi) {
    theta_ = theta;
    phi_ = phi;
  }

  auto Id() const noexcept { return id_; }

  Vector<T, 3> ToVector() const;

  bool IsAbsolute() const { return relative_point_ == nullptr; }

  void SetRelativePoint(PointPtr<T> relative_point, Matrix3<T> conversion_mat) {
    assert(relative_point != nullptr);
    relative_point_ = relative_point;
    conversion_mat_ = conversion_mat;
  }

  T DistanceTo(const Point<T> &other) const;

  /**
   * 相対位置の連なりを一つにまとめる
   */
  void ShortenRelativePath();

 private:
  bool RelativePathIsShortened() const { return IsAbsolute() || relative_point_->IsAbsolute(); }

  size_t id_;
  T theta_;
  T phi_;
  PointPtr<T> relative_point_;
  std::optional<Matrix3<T>> conversion_mat_;
};

namespace {

template <typename T>
std::tuple<T, T, T> InitXYZ(T x, T y, uint f, T d) {
  const T xd = (d - x * T(2.)) / (d + T(2.));
  const T yd = (d - y * T(2.)) / (d + T(2.));
  switch (f) {
    case 0:
      return std::make_tuple(xd, yd, 1.);
    case 1:
      return std::make_tuple(1., xd, yd);
    case 2:
      return std::make_tuple(yd, 1., xd);
    case 3:
      return std::make_tuple(-xd, -1., -yd);
    case 4:
      return std::make_tuple(-yd, -xd, -1.);
    case 5:
      return std::make_tuple(-1., -yd, -xd);
    default:
      std::exit(-1);
  }
}

}  // namespace

template <typename T>
Point<T>::Point(size_t id, uint x, uint y, uint f, uint d)
    : id_(id), relative_point_(nullptr), conversion_mat_(std::nullopt) {
  const auto [xx, yy, zz] = InitXYZ<T>(T(x), T(y), f, T(d));
  const auto r = std::hypot(xx, yy, zz);
  theta_ = std::acos(zz / r);
  phi_ = std::atan2(yy, xx);
}

template <typename T>
Vector<T, 3> Point<T>::ToVector() const {
  if (relative_point_) {
    return *conversion_mat_ * relative_point_->ToVector();
  } else {
    return Vector<T, 3>(std::sin(theta_) * std::cos(phi_), std::sin(theta_) * std::sin(phi_), std::cos(theta_));
  }
}

template <typename T>
T Point<T>::DistanceTo(const Point<T> &other) const {
  const auto diff = ToVector() - other.ToVector();
  return diff.norm();
}

template <typename T>
void Point<T>::ShortenRelativePath() {
  if (RelativePathIsShortened()) return;
  relative_point_->ShortenRelativePath();
  conversion_mat_ = *conversion_mat_ * *relative_point_->conversion_mat_;
  relative_point_ = relative_point_->relative_point_;
  assert(RelativePathIsShortened());
}

}  // namespace polyhedron
