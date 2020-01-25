#include <Eigen/Core>

namespace app {

template <typename Type, int Size>
using Vector = Eigen::Matrix<Type, Size, 1>;

template <typename Type>
using Matrix3 = Eigen::Matrix<Type, 3, 3>;

}  // namespace app
