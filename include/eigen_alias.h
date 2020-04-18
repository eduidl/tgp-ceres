#include <Eigen/Core>

namespace polyhedron {

template <typename Type, int Size>
using Vector = Eigen::Matrix<Type, Size, 1>;

template <typename Type>
using Matrix3 = Eigen::Matrix<Type, 3, 3>;

}  // namespace polyhedron
