#include <cassert>

#include "line.h"
#include "point.h"

namespace app {

template <typename T>
LineSegment<T>::LineSegment(PointPtr<T> begin, PointPtr<T> end) : begin_(begin), end_(end) {
  assert(begin_ != nullptr && end_ != nullptr && begin_->Id() != end_->Id());
}

template <typename T>
T LineSegment<T>::Length() const {
  return begin_->DistanceTo(*end_);
}

template <typename T>
bool LineSegment<T>::operator==(const LineSegment &rhs) const {
  return Ids() == rhs.Ids();
}

template <typename T>
bool LineSegment<T>::operator>(const LineSegment &rhs) const {
  return Ids() > rhs.Ids();
}

template <typename T>
bool LineSegment<T>::operator<(const LineSegment &rhs) const {
  return Ids() < rhs.Ids();
}

template <typename T>
std::tuple<size_t, size_t> LineSegment<T>::Ids() const {
  const auto max_id = std::max(begin_->Id(), end_->Id());
  const auto min_id = std::min(begin_->Id(), end_->Id());
  return std::make_tuple(max_id, min_id);
}

template class LineSegment<double>;
template class LineSegment<float>;

}  // namespace app
