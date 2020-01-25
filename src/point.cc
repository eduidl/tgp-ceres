#include <cassert>

#include "point.h"

namespace app {

namespace {

template <typename T>
std::tuple<T, T, T> InitXYZ(uint x, uint y, uint f, uint d) {
  const T xd = T(1) - T((x + 1) * 2) / T(d + 2);
  const T yd = T(1) - T((y + 1) * 2) / T(d + 2);
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
  const auto [xx, yy, zz] = InitXYZ<T>(x, y, f, d);
  const auto r = std::hypot(xx, yy, zz);
  theta_ = std::acos(zz / r);
  phi_ = std::atan2(yy, xx);
}

template <typename T>
std::tuple<T, T> Point<T>::GetParams() const {
  return std::make_tuple(theta_, phi_);
}

template <typename T>
void Point<T>::SetParams(T theta, T phi) {
  theta_ = theta;
  phi_ = phi;
}

template <typename T>
size_t Point<T>::Id() const noexcept {
  return id_;
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
bool Point<T>::IsAbsolute() const {
  return relative_point_ == nullptr;
}

template <typename T>
void Point<T>::SetRelativePoint(PointPtr<T> relative_point, Matrix3<T> conversion_mat) {
  assert(relative_point != nullptr);
  relative_point_ = relative_point;
  conversion_mat_ = conversion_mat;
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

template <typename T>
bool Point<T>::RelativePathIsShortened() const {
  return IsAbsolute() || relative_point_->IsAbsolute();
}

template class Point<double>;
template class Point<float>;

}  // namespace app
