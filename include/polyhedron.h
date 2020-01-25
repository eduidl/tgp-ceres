#pragma once

#include <memory>
#include <string>
#include <vector>

#include "face.h"
#include "line.h"

namespace app {

template <typename T>
class Point;

template <typename T>
using PointPtr = std::shared_ptr<Point<T>>;

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

  size_t EdgeSize() const { return edges_.size(); }
  T EdgeLength(size_t i) const { return edges_[i].Length(); }
  T EdgeLengthMean() const { return edge_length_mean_; }
  size_t AbsPointSize() const { return abs_point_indices_.size(); }
  size_t DiagonalSize() const { return diagonals_.size(); }
  T DiagonalLength(size_t i) const { return diagonals_[i].Length(); }

 private:
  inline size_t Id(uint x, uint y, uint f) const;
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

}  // namespace app
