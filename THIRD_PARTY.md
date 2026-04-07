# Third-Party Components

This repository contains original project code, repository-specific extensions, and bundled third-party source code.

## Project-Level Provenance

### `pdpcFeature`

- Derived from the upstream project `Plane-Detection-Point-Cloud`
- Upstream repository: <https://github.com/ThibaultLejemble/Plane-Detection-Point-Cloud>
- This directory contains repository-specific extensions for logging, intermediate export, full segmentation export, and `.vg` export
- The license for this directory is documented in [`pdpcFeature/LICENSE`](./pdpcFeature/LICENSE)

Bundled source directories inside `pdpcFeature` include:

- `src/Eigen`
- `src/Ponca`

### `DCI`

- Contains the downstream DCI code developed for this paper project
- The license for this directory is documented in [`DCI/LICENSE`](./DCI/LICENSE)

Bundled source directories inside `DCI` include:

- `module/3rd/spdlog`
- `module/3rd/cxxopts-2.2.1`

### `pointSample`

- Contains the weighted point-sampling code and comparison tooling used with DCI-derived descriptors
- The license for this directory is documented in [`pointSample/LICENSE`](./pointSample/LICENSE)

Third-party dependencies used by `pointSample` include:

- `Eigen3` as an external system dependency
- `OpenMP` as a compiler/runtime dependency
- `cxxopts` as a header-only dependency fetched by CMake when not available locally

## User Responsibility

If you reuse or redistribute this repository, review:

- the license file in the subproject you use
- the license files shipped with bundled third-party code
- the license terms of external system dependencies such as CGAL, Boost, PCL, Eigen3, and OpenCV

This file is a release note for provenance and packaging clarity. It is not a substitute for the original license texts.
