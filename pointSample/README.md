# pointSample

`pointSample` is the weighted point-sampling submodule used by the `pointDCI` paper code.

It serves two purposes:

- run weighted farthest point sampling on point clouds or mesh vertices
- reproduce the weighted-sampling comparisons discussed in the paper, especially the strategies based on DCI-derived `consistency` and `stability` values

This directory is a research-oriented companion module for [`../DCI`](../DCI). It is not a standalone end-to-end pipeline.

## Relationship To DCI

The main `DCI` pipeline computes per-point descriptors such as `consistency` and `stability`.

`pointSample` consumes:

- a point cloud or mesh
- optional per-point `consistency` values
- optional per-point `stability` values

The DCI-derived files are then used as sampling weights for comparison experiments.

## Features

- weighted FPS with pluggable strategies
- optional octree downsampling before FPS
- `.ply` point cloud input
- `.obj` mesh-vertex input for RGB-driven sampling
- colored sidecar exports for qualitative analysis

Implemented weighting strategies:

- `uniform`
- `curvature`
- `density`
- `color-intensity`
- `consistency`
- `stability`
- `consistency_stability`

## Build Requirements

- CMake >= 3.16
- a C++17 compiler
- OpenMP
- Eigen3

Header-only dependency:

- `cxxopts`

If `cxxopts` is already available in the surrounding monorepo, `pointSample` will reuse that copy automatically.

If `cxxopts` is not already available on your system include path, CMake will fetch it at configure time.

## Build

```bash
cmake -S pointSample -B build/pointSample -DCMAKE_BUILD_TYPE=Release
cmake --build build/pointSample -j
```

The executable will be generated at `build/pointSample/pointSample`.

Run the minimal smoke tests with:

```bash
ctest --test-dir build/pointSample --output-on-failure
```

## Usage

```bash
./build/pointSample/pointSample --input path/to/model.ply --output sampled.ply
```

Print the full CLI reference with:

```bash
./build/pointSample/pointSample --help
```

## Common Examples

Uniform FPS:

```bash
./build/pointSample/pointSample \
  --input data/model.ply \
  --output out_uniform.ply \
  --num 4096 \
  --strategy uniform
```

Consistency-weighted FPS using DCI output:

```bash
./build/pointSample/pointSample \
  --input data/model.ply \
  --output out_consistency.ply \
  --num 4096 \
  --strategy consistency \
  --consistency data/consistency.txt
```

Joint consistency/stability weighting:

```bash
./build/pointSample/pointSample \
  --input data/model.ply \
  --output out_cs.ply \
  --num 4096 \
  --strategy consistency_stability \
  --consistency data/consistency.txt \
  --stability data/stability.txt \
  --gamma 1.0
```

OBJ mesh vertex sampling with RGB-derived intensity and octree pre-downsampling:

```bash
./build/pointSample/pointSample \
  --input data/model.obj \
  --output out_mesh.ply \
  --colorMesh true \
  --downsample octree \
  --octree-level 7
```

## Input Expectations

- `--input` accepts `.ply` point clouds or `.obj` meshes
- `--consistency` and `--stability` expect plain-text files with one floating-point value per line
- when supplied, the number of weights must match the input point count exactly

For `color-intensity`, the input must carry color information.

For `consistency`, `stability`, and `consistency_stability`, the corresponding DCI-generated text files must be provided explicitly.

## Outputs

The main output is the sampled point cloud written to `--output`.

Depending on options, the tool may also export sidecar files:

- full-cloud colored visualizations for qualitative inspection
- a downsampled intermediate point cloud when octree mode is used
- gated or strategy-specific sidecar outputs derived from the output filename

## Research Scope

This module is released as research code prepared for reproducibility and inspection. It is intended to document the weighted-sampling portion of the paper implementation, not to provide a polished general-purpose library.

## License

This directory is distributed under the terms described in [`LICENSE`](./LICENSE).

Third-party dependency provenance is summarized in [`../THIRD_PARTY.md`](../THIRD_PARTY.md).
