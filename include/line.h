#pragma once

#include <memory>
#include <tuple>

namespace app {

template <typename T>
class Point;

template <typename T>
using PointPtr = std::shared_ptr<Point<T>>;

template <typename T>
class LineSegment {
 public:
  LineSegment(PointPtr<T> begin, PointPtr<T> end);
  virtual ~LineSegment() = default;

  T Length() const;
  bool operator==(const LineSegment &rhs) const;
  bool operator<(const LineSegment &rhs) const;
  bool operator>(const LineSegment &rhs) const;
  std::tuple<size_t, size_t> Ids() const;

 private:
  PointPtr<T> begin_;
  PointPtr<T> end_;
};

}  // namespace app
