#pragma once

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "eigen_alias.h"
#include "face.h"
#include "line.h"
#include "point.h"

namespace polyhedron {

namespace {

inline auto CalcN(uint h, uint k) { return 6 * (h * h + k * k); }

inline auto CalcD(uint h, uint k) { return h + k - 1; }

inline auto Index(uint x, uint y, uint f, uint d) { return x + y * (d + 1) + f * (d + 1) * (d + 1); }
}  // namespace

template <typename T>
class Polyhedron {
 public:
  Polyhedron(uint h, uint k);
  virtual ~Polyhedron() = default;
  Polyhedron(const Polyhedron& o) = delete;
  Polyhedron& operator=(const Polyhedron& o) = delete;
  Polyhedron(Polyhedron&& o) = default;
  Polyhedron& operator=(Polyhedron&& o) = default;

  void GetInitialParams(std::vector<T>& theta, std::vector<T>& phi) const;
  void UpdateParams(T const* const theta, T const* const phi);
  void DumpTs(std::string& fname) const;

  const auto& Edges() const { return edges_; }
  const auto& Diagonals() const { return diagonals_; }
  auto EdgeLengthMean() const { return edge_length_mean_; }
  auto AbsPointSize() const { return abs_point_indices_.size(); }

 private:
  auto Id(uint x, uint y, uint f) const { return id_convert_map_.at(Index(x, y, f, d_)); }
  void CreatePoints();
  void CreateSquares();
  void CreateTriangles();
  void CreateEdges();
  void CreateDiagonals();
  T CalcEdgeLengthMean();

  uint h_;
  uint k_;
  uint n_;
  uint d_;
  std::vector<PointPtr<T>> points_;
  std::vector<Square<T>> squares_;
  std::vector<Triangle<T>> triangles_;
  std::vector<LineSegment<T>> edges_;
  std::vector<LineSegment<T>> diagonals_;
  std::vector<size_t> id_convert_map_;
  std::vector<size_t> abs_point_indices_;

  T edge_length_mean_;
};

namespace {

template <typename T>
static const Matrix3<T> kRotateX = (Matrix3<T>() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
template <typename T>
static const Matrix3<T> kRotateY = (Matrix3<T>() << 0, 0, 1, 0, 1, 0, -1, 0, 0).finished();
template <typename T>
static const Matrix3<T> kRotateZ = (Matrix3<T>() << 0, -1, 0, 1, 0, 0, 0, 0, 1).finished();

template <typename T>
Matrix3<T> MatPower(const Matrix3<T> mat, uint num) {
  if (num == 0) {
    return Matrix3<T>::Identity();
  }
  auto ret = mat;
  for (uint i = 1; i < num; ++i) {
    ret *= mat;
  }
  return ret;
}

auto CreateIdConvertMap(uint h, uint k) {
  const auto d = CalcD(h, k);
  const size_t size_including_duplicates = 6 * (d + 1) * (d + 1);
  std::vector<int> equiv_points(6 * (d + 1) * (d + 1), -1);
  const auto index = [=](uint x, uint y, uint f) { return Index(x, y, f, d); };
  if (k > 0) {
    // 辺の部分の重なりを見ているはず……（記憶にない）
    for (uint dh = 0; dh < h; dh++) {
      for (uint dk = 0; dk < k; dk++) {
        equiv_points[index(0 + dh, 0 + dk, 1)] = index(k - 1 - dk, k + dh, 0);
        equiv_points[index(0 + dh, 0 + dk, 2)] = index(k - 1 - dk, k + dh, 1);
        equiv_points[index(k - 1 - dk, k + dh, 2)] = index(0 + dh, 0 + dk, 0);  // swap
        equiv_points[index(0 + dh, 0 + dk, 5)] = index(k - 1 - dk, k + dh, 3);
        equiv_points[index(k - 1 - dk, k + dh, 4)] = index(0 + dh, 0 + dk, 3);  // swap
        equiv_points[index(k - 1 - dk, k + dh, 5)] = index(0 + dh, 0 + dk, 4);  // swap

        equiv_points[index(d - dh, d - dk, 3)] = index(k + dh, h + dk, 0);
        equiv_points[index(d - dh, d - dk, 4)] = index(k + dh, h + dk, 1);
        equiv_points[index(d - dh, d - dk, 5)] = index(k + dh, h + dk, 2);

        equiv_points[index(d - dk, 0 + dh, 5)] = index(h + dk, h - 1 - dh, 0);
        equiv_points[index(d - dk, 0 + dh, 3)] = index(h + dk, h - 1 - dh, 1);
        equiv_points[index(d - dk, 0 + dh, 4)] = index(h + dk, h - 1 - dh, 2);
      }
    }
  }

  std::vector<size_t> id_convert_map;
  id_convert_map.reserve(size_including_duplicates);
  size_t next_id = 0;
  for (const auto equiv_point : equiv_points) {
    if (equiv_point < 0) {
      id_convert_map.push_back(next_id);
      ++next_id;
    } else {
      assert(id_convert_map[equiv_point] < next_id);
      id_convert_map.push_back(id_convert_map[equiv_point]);
    }
  }
  assert(id_convert_map.size() == size_including_duplicates);
  return id_convert_map;
}

struct XY {
  uint x;
  uint y;

  bool Sum() const { return x + y; }
  bool PriorThan(XY& other) { return Sum() < other.Sum() || (Sum() == other.Sum() && y < other.y); }
};

template <typename T>
std::tuple<XY, Matrix3<T>> GetRotation(uint x, uint y, uint f, uint d) {
  switch (f) {
    case 0: {
      std::array<XY, 4> xy_set{XY{x, y}, XY{y, d - x}, XY{d - x, d - y}, XY{d - y, x}};
      uint argmin = 0;
      for (uint i = 1; i < 4; ++i) {
        if (xy_set[i].PriorThan(xy_set[argmin])) {
          argmin = i;
        }
      }
      return std::make_tuple(xy_set[argmin], MatPower(kRotateZ<T>, argmin));
    }
    case 1:
      return std::make_tuple(XY{d - y, x}, kRotateY<T>);
    case 2:
      return std::make_tuple(XY{y, d - x}, MatPower(kRotateX<T>, 3));
    case 3:
      return std::make_tuple(XY{d - x, d - y}, kRotateX<T>);
    case 4:
      return std::make_tuple(XY{y, d - x}, MatPower(kRotateY<T>, 2));
    case 5:
      return std::make_tuple(XY{d - x, d - y}, MatPower(kRotateY<T>, 3));
    default:
      exit(-1);
  }
}

}  // namespace

template <typename T>
Polyhedron<T>::Polyhedron(uint h, uint k)
    : h_(h), k_(k), n_(CalcN(h, k)), d_(CalcD(h, k)), points_(CalcN(h, k)), id_convert_map_(CreateIdConvertMap(h, k)) {
  assert(h >= k);
  std::cout << "h = " << h_ << ", k = " << k_ << ", n = " << n_ << ", d = " << d_ << std::endl;
  CreatePoints();
  CreateSquares();
  CreateTriangles();
  CreateEdges();
  CreateDiagonals();
  edge_length_mean_ = CalcEdgeLengthMean();
}

template <typename T>
void Polyhedron<T>::GetInitialParams(std::vector<T>& theta, std::vector<T>& phi) const {
  for (size_t i = 0; i < abs_point_indices_.size(); ++i) {
    const auto [theta_i, phi_i] = points_[abs_point_indices_[i]]->GetParams();
    theta[i] = theta_i;
    phi[i] = phi_i;
  }
}

template <typename T>
void Polyhedron<T>::UpdateParams(T const* const theta, T const* const phi) {
  for (size_t i = 0; i < abs_point_indices_.size(); ++i) {
    points_[abs_point_indices_[i]]->SetParams(theta[i], phi[i]);
  }
  edge_length_mean_ = CalcEdgeLengthMean();
}

template <typename T>
void Polyhedron<T>::DumpTs(std::string& fname) const {
  std::ofstream ofs;
  ofs.open(fname, std::ios::out);
  if (ofs.fail()) {
    std::cout << "failed to open " << fname << std::endl;
    return;
  }
  ofs << "import { Point, Edge} from \"./types\";\n\n";
  ofs << "export const POINTS: Point[] = [\n";
  for (const auto& point : points_) {
    const auto vec = point->ToVector();
    ofs << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "],\n";
  }
  ofs << "]\n\n";
  ofs << "export const EDGES: Edge[] = [\n";
  for (const auto& edge : edges_) {
    const auto [id1, id2] = edge.Ids();
    ofs << "[" << id1 << ", " << id2 << "],\n";
  }
  ofs << "]\n";
}

template <typename T>
void Polyhedron<T>::CreatePoints() {
  points_.resize(n_);
  // 平面0におけるC4対称性を利用
  // 1-5面はそれぞれ適切に回転させると0面に重なることを利用
  for (uint f = 0; f < 6; ++f) {
    for (uint y = 0; y <= d_; ++y) {
      for (uint x = 0; x <= d_; ++x) {
        const auto id = Id(x, y, f);
        if (points_[id] != nullptr) continue;
        points_[id] = std::make_shared<Point<T>>(id, x, y, f, d_);
        const auto [rot_from, rot_mat] = GetRotation<T>(x, y, f, d_);
        const auto rot_from_id = Id(rot_from.x, rot_from.y, 0);
        if (id == rot_from_id) continue;
        points_[id]->SetRelativePoint(points_[rot_from_id], rot_mat);
      }
    }
  }

  assert(std::all_of(points_.begin(), points_.end(), [](const auto& v) { return v != nullptr; }));

  for (size_t i = 0; i < points_.size(); ++i) {
    points_[i]->ShortenRelativePath();
    if (points_[i]->IsAbsolute()) abs_point_indices_.push_back(i);
  }
}

template <typename T>
void Polyhedron<T>::CreateSquares() {
  const auto append_square = [&](size_t id1, size_t id2, size_t id3, size_t id4) {
    squares_.emplace_back(points_[id1], points_[id2], points_[id3], points_[id4]);
  };
  if (k_ == 0) {
    squares_.reserve(6 * d_ * d_ + 12 * d_);
  } else {
    squares_.reserve(6 * d_ * d_);
  }
  // 正方形d*d枚の部分
  for (uint f = 0; f < 6; ++f) {
    for (uint y = 0; y < d_; ++y) {
      for (uint x = 0; x < d_; ++x) {
        append_square(Id(x, y, f), Id(x + 1, y, f), Id(x + 1, y + 1, f), Id(x, y + 1, f));
      }
    }
  }
  if (k_ == 0) {
    // 辺の部分
    for (uint i = 0; i < d_; ++i) {
      append_square(Id(i, 0, 1), Id(i + 1, 0, 1), Id(0, i + 1, 0), Id(0, i, 0));
      append_square(Id(i, 0, 0), Id(i + 1, 0, 0), Id(0, i + 1, 2), Id(0, i, 2));
      append_square(Id(i, 0, 2), Id(i + 1, 0, 2), Id(0, i + 1, 1), Id(0, i, 1));

      append_square(Id(i, d_, 0), Id(i + 1, d_, 0), Id(d_ - (i + 1), d_, 3), Id(d_ - i, d_, 3));
      append_square(Id(i, d_, 1), Id(i + 1, d_, 1), Id(d_ - (i + 1), d_, 4), Id(d_ - i, d_, 4));
      append_square(Id(i, d_, 2), Id(i + 1, d_, 2), Id(d_ - (i + 1), d_, 5), Id(d_ - i, d_, 5));

      append_square(Id(d_, i, 0), Id(d_, i + 1, 0), Id(d_, d_ - (i + 1), 5), Id(d_, d_ - i, 5));
      append_square(Id(d_, i, 1), Id(d_, i + 1, 1), Id(d_, d_ - (i + 1), 3), Id(d_, d_ - i, 3));
      append_square(Id(d_, i, 2), Id(d_, i + 1, 2), Id(d_, d_ - (i + 1), 4), Id(d_, d_ - i, 4));

      append_square(Id(d_ - i, 0, 3), Id(d_ - (i + 1), 0, 3), Id(0, d_ - (i + 1), 4), Id(0, d_ - i, 4));
      append_square(Id(d_ - i, 0, 4), Id(d_ - (i + 1), 0, 4), Id(0, d_ - (i + 1), 5), Id(0, d_ - i, 5));
      append_square(Id(d_ - i, 0, 5), Id(d_ - (i + 1), 0, 5), Id(0, d_ - (i + 1), 3), Id(0, d_ - i, 3));
    }
  }
}

template <typename T>
void Polyhedron<T>::CreateTriangles() {
  const auto append_triangle = [&](size_t id1, size_t id2, size_t id3) {
    triangles_.emplace_back(points_[id1], points_[id2], points_[id3]);
  };
  // 三角形
  if (k_ == 0) {
    // 全体を立方体のように扱ったときの頂点にあたる
    append_triangle(Id(0, 0, 0), Id(0, 0, 1), Id(0, 0, 2));  // A
    append_triangle(Id(0, 0, 3), Id(0, 0, 4), Id(0, 0, 5));  // G

    append_triangle(Id(0, d_, 0), Id(d_, 0, 1), Id(d_, d_, 3));  // B
    append_triangle(Id(0, d_, 1), Id(d_, 0, 2), Id(d_, d_, 4));  // E
    append_triangle(Id(0, d_, 2), Id(d_, 0, 0), Id(d_, d_, 5));  // D

    append_triangle(Id(d_, d_, 0), Id(d_, 0, 5), Id(0, d_, 3));  // C
    append_triangle(Id(d_, d_, 1), Id(d_, 0, 3), Id(0, d_, 4));  // F
    append_triangle(Id(d_, d_, 2), Id(d_, 0, 4), Id(0, d_, 5));  // H
  } else {
    append_triangle(Id(h_ - 1, 0, 0), Id(h_, 0, 0), Id(d_, h_, 5));
    append_triangle(Id(d_, h_ - 1, 0), Id(d_, h_, 0), Id(h_ - 1, 0, 5));
    append_triangle(Id(k_ - 1, d_, 0), Id(k_, d_, 0), Id(h_, 0, 1));
    append_triangle(Id(0, k_ - 1, 0), Id(0, k_, 0), Id(0, k_, 1));

    append_triangle(Id(h_ - 1, 0, 4), Id(h_, 0, 4), Id(d_, h_, 2));
    append_triangle(Id(d_, h_ - 1, 4), Id(d_, h_, 4), Id(h_ - 1, 0, 2));
    append_triangle(Id(k_ - 1, d_, 4), Id(k_, d_, 4), Id(h_, 0, 3));
    append_triangle(Id(0, k_ - 1, 4), Id(0, k_, 4), Id(0, k_, 3));
  }
}

template <typename T>
void Polyhedron<T>::CreateEdges() {
  edges_.reserve(2 * n_);
  if (squares_.empty()) {
    for (const auto& triangle : triangles_) {
      const auto local_edges = triangle.Edges();
      edges_.insert(edges_.end(), local_edges.begin(), local_edges.end());
    }
  } else {
    for (const auto& square : squares_) {
      const auto local_edges = square.Edges();
      edges_.insert(edges_.end(), local_edges.begin(), local_edges.end());
    }
  }
  std::sort(edges_.begin(), edges_.end());
  edges_.erase(std::unique(edges_.begin(), edges_.end()), edges_.end());
  assert(edges_.size() == 2 * n_);
}

template <typename T>
void Polyhedron<T>::CreateDiagonals() {
  diagonals_.reserve(2 * squares_.size());
  for (const auto& square : squares_) {
    const auto local_diagonals = square.Diagonals();
    diagonals_.insert(diagonals_.end(), local_diagonals.begin(), local_diagonals.end());
  }
  assert(diagonals_.size() == 2 * squares_.size());
}

template <typename T>
T Polyhedron<T>::CalcEdgeLengthMean() {
  const auto sum = std::accumulate(edges_.begin(), edges_.end(), T(0.),
                                   [](T acc, LineSegment<T> edge) { return acc + edge.Length(); });
  return sum / static_cast<T>(edges_.size());
}

}  // namespace polyhedron
