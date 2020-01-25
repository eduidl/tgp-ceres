#include <cassert>

#include <Eigen/Dense>

#include "face.h"
#include "point.h"

namespace app {

template <typename T>
Square<T>::Square(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3, PointPtr<T> p4) : points_({p1, p2, p3, p4}) {
  assert(p1 != nullptr && p2 != nullptr && p3 != nullptr && p4 != nullptr);
}

template <typename T>
std::vector<LineSegment<T>> Square<T>::Edges() const {
  return std::vector<LineSegment<T>>{
      {points_[0], points_[1]}, {points_[1], points_[2]}, {points_[2], points_[3]}, {points_[3], points_[0]}};
}

template <typename T>
std::vector<LineSegment<T>> Square<T>::Diagonals() const {
  return std::vector<LineSegment<T>>{{points_[0], points_[2]}, {points_[1], points_[3]}};
}

template class Square<double>;
template class Square<float>;

template <typename T>
Triangle<T>::Triangle(PointPtr<T> p1, PointPtr<T> p2, PointPtr<T> p3) : points_({p1, p2, p3}) {
  assert(p1 != nullptr && p2 != nullptr && p3 != nullptr);
}

template <typename T>
std::vector<LineSegment<T>> Triangle<T>::Edges() const {
  return std::vector<LineSegment<T>>{{points_[0], points_[1]}, {points_[1], points_[2]}, {points_[2], points_[0]}};
}

template class Triangle<double>;
template class Triangle<float>;

}  // namespace app
