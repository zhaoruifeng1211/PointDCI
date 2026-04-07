# pdpcFeature

`pdpcFeature` is a modified version of the upstream project [Plane-Detection-Point-Cloud](https://github.com/ThibaultLejemble/Plane-Detection-Point-Cloud), focused on multi-scale planar structure analysis, segmentation result export, and post-processing for 3D point clouds.

This repository is not intended to restate the original paper implementation. Its main purpose is to expose the extensions we added on top of the upstream codebase, including runtime logging, intermediate result export, full persistence-based segmentation export, and `.vg` output support.

## Project Scope

This repository can be built and run independently for the core pipeline, but it is only one algorithmic module within our broader project.

- This repository covers multi-scale feature computation, segmentation, and post-processing export for point clouds.
- The parent directory may contain additional projects or data folders. They are not part of this repository's open-source scope.
- Some scripts under `process/` are lightweight utility wrappers and may require local path adjustments before general use.

## Main Changes Relative to the Upstream Project

On top of the original "multi-scale feature computation + segmentation + post-processing" pipeline, this repository adds the following capabilities.

### 1. Runtime Logging

- Added runtime measurement to `pdpcComputeMultiScaleFeatures` and `pdpcSegmentation`
- Prints total runtime and appends it to `runtime_log.txt` in the output directory

### 2. Multi-Scale Geometric Variation Export

- Saves per-point, per-scale geometric variation information during the feature computation stage
- Adds new outputs:
  - `*_geometric_variation.txt`
  - `*_iterRadius.txt`

### 3. Per-Scale Smoothed / Projected Point Cloud Export

- Exports the projected point cloud produced at each scale during iterative processing
- Example outputs:
  - `*_0scaleProj.ply`
  - `*_1scaleProj.ply`

These outputs are useful for debugging scale behavior, checking smoothing results, and visualizing intermediate states.

### 4. `.vg` Export

- Adds `.vg` export support during post-processing
- Groups points by segmentation label, fits planes, and writes structured output for downstream tools
- The current implementation includes `saveVG(...)` and `saveVGFast(...)`

### 5. Full Persistence-Based Segmentation Output

- Adds an additional "full segmentation" export path that fills previously unlabeled points after persistence thresholding
- Example outputs:
  - `*_FullSegPersxxx.ply`
  - `*_FullSegPersxxx.vg`

### 6. Label and Relabel Text Export

- Adds plain-text outputs for both the original labels and the completed relabeling
- Example outputs:
  - `*_PersLabelXXX.txt`
  - `*_PersReLabelXXX.txt`

## Requirements

Linux is the recommended environment for building and running this repository.

### Build Dependencies

- CMake 3.x
- A C++14-compatible compiler
- CGAL
- Boost

### Bundled Third-Party Source Code

- Eigen
- Ponca

### Reference Environment

This repository evolves from the upstream project. The original project README reports tests on Ubuntu, Debian, and macOS.  
Because this version adds CGAL- and Boost-related build requirements, a modern Linux environment is the recommended default.

## Build

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

The build generates the following main executables:

- `pdpcComputeMultiScaleFeatures`
- `pdpcSegmentation`
- `pdpcPostProcess`

For a one-command local build helper, you can also run:

```bash
./replicate.sh
```

That helper only configures and builds the public executables. It does not install system packages automatically.

## Basic Workflow

Assume the input point cloud is `mycloud.ply`.

### 1. Multi-Scale Feature Computation

```bash
./build/pdpcComputeMultiScaleFeatures -v -i mycloud.ply -o mycloud
```

This step generates the multi-scale features and related intermediate outputs.

### 2. Multi-Scale Segmentation

```bash
./build/pdpcSegmentation -v -i mycloud.ply -s mycloud_scales.txt -f mycloud_features.txt -o mycloud
```

This step performs multi-scale planar region growing using the outputs from the previous stage.

### 3. Post-Processing and Export

```bash
./build/pdpcPostProcess -v -i mycloud.ply -s mycloud_seg.txt -c mycloud_comp.txt -o results -pers 10 15 20 25 -col
```

`pdpcPostProcess` supports three main modes:

- `-range`: export components within persistence ranges
- `-pers`: export components above persistence thresholds
- `-scales`: export components selected at specific scales

Common examples:

```bash
./build/pdpcPostProcess -v -i mycloud.ply -s mycloud_seg.txt -c mycloud_comp.txt -o results_range -range 0 9 10 19
./build/pdpcPostProcess -v -i mycloud.ply -s mycloud_seg.txt -c mycloud_comp.txt -o results_pers -pers 10 15 20 25
./build/pdpcPostProcess -v -i mycloud.ply -s mycloud_seg.txt -c mycloud_comp.txt -o results_scales -scales 5 10 15 20
```

With `-col`, the pipeline also exports colorized `.ply` files. In some modes it also writes `.vg` files.

## Output Files

In addition to the upstream outputs, this repository adds the following result files:

- `*_geometric_variation.txt`
- `*_iterRadius.txt`
- `*_NscaleProj.ply`
- `*_PersLabelXXX.txt`
- `*_PersReLabelXXX.txt`
- `*_FullSegPersXXX.ply`
- `*_FullSegPersXXX.vg`
- `*.vg`
- `runtime_log.txt`

These outputs are mainly intended for:

- preserving intermediate multi-scale analysis data
- tracking label changes across post-processing stages
- inspecting smoothing and projection behavior at different scales
- connecting the results to downstream external workflows

## Notes on the `process/` Scripts

The scripts under `process/` are mainly intended for batch processing and lightweight format conversion.

Before using them, note that:

- they assume the executables are available under `../build/`
- some of them depend on a fixed directory layout
- some are best treated as convenience wrappers rather than stable public interfaces

If you only need the core functionality, calling the executables under `build/` directly is recommended.

## Relation to the Upstream Project

This repository is derived from:

- Upstream repository: <https://github.com/ThibaultLejemble/Plane-Detection-Point-Cloud>
- Related paper: <https://hal.archives-ouvertes.fr/hal-02490721/document>

Original paper:

> Thibault Lejemble, Claudio Mura, Loic Barthe, Nicolas Mellado.  
> Persistence Analysis of Multi-scale Planar Structure Graph in Point Clouds.  
> Computer Graphics Forum, Eurographics 2020.

This repository is not an official release from the original authors. It is a locally modified and extended version built on top of their codebase.

## License

This directory is distributed under the terms described in [`LICENSE`](./LICENSE).

This repository uses or includes code and dependencies from upstream and third-party components, including but not limited to:

- Plane-Detection-Point-Cloud
- Eigen
- Ponca
- CGAL
- Boost

If you use, redistribute, or modify this repository, you should also review the licenses and usage terms of the corresponding upstream and third-party components.
