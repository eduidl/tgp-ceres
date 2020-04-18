#pragma once

#include <cassert>
#include <tuple>

#include "point.h"

namespace polyhedron {

template <typename T>
class LineSegment {
 public:
  LineSegment(PointPtr<T> begin, PointPtr<T> end) : begin_(begin), end_(end) {
    assert(begin_ != nullptr && end_ != nullptr && begin_->Id() != end_->Id());
  }
  virtual ~LineSegment() = default;

  T Length() const { return begin_->DistanceTo(*end_); }

  bool operator==(const LineSegment &rhs) const { return Ids() == rhs.Ids(); }
  bool operator<(const LineSegment &rhs) const { return Ids() > rhs.Ids(); }
  bool operator>(const LineSegment &rhs) const { return Ids() < rhs.Ids(); }

  auto Ids() const {
    const auto max_id = std::max(begin_->Id(), end_->Id());
    const auto min_id = std::min(begin_->Id(), end_->Id());
    return std::make_tuple(max_id, min_id);
  }

 private:
  PointPtr<T> begin_;
  PointPtr<T> end_;
};

}  // namespace polyhedron
