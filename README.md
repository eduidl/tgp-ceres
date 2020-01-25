# TGP Ceres

TGP is short for Tetravalent Goldberg Polyhedra.

## General

This program enable us to calculate vertex coordinates of tetravalent Goldberg polyhedra.  
If you want to know about tetravalent Goldberg polyhedra, check following paper or articles.
- https://www.nature.com/articles/nature20771 (not free)
- http://www.jst.go.jp/pr/announce/20161222/index.html (japanese)
- https://www.chem-station.com/blog/2016/12/Goldberg.html (japanese)

You can see examples of results [here](https://eduidl.github.io/polyhedron/).

NOTE: This program cannot distinguish chirarity.  

## Requirements

- Cmake
- Ceres Solver
  - http://ceres-solver.org/installation.html
- Node.js (only for visualization)

## Build

```sh
mkdir build && cd build
cmale .. && make -j
```

## Run

Both `h` and `k` should be intergers which are not less than zero. The meanings of `h` and `k` is described in paper and articles I mentioned.

```sh
./main [h] [k]
```

### Visualization

You can check last result you have got.

```sh
npm i
npm run start
```
