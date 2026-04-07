# pointDCI

`pointDCI` is a research monorepo for our paper code on point-cloud detail contribution analysis.

Current paper title:
`PointDCI: Quantifying Multi-scale Structural Significance in 3D Point Clouds`

Authors:
Ruifeng Zhao, Yubo Men, Yongmei Liu, Chaoguang Men, Jin Li, Zeyu Tian

Current submission status:
Submitted to `The Visual Computer`

It packages three connected stages in one repository:

- preprocessing with `pdpcFeature`
- descriptor generation with `DCI`
- weighted sampling experiments with `pointSample`

This repository currently contains three related subprojects:

- [`pdpcFeature`](./pdpcFeature): the upstream PDPC-based feature extraction and segmentation stage, with our local extensions for export, logging, and `.vg` generation
- [`DCI`](./DCI): our downstream DCI stage for consistency/stability analysis and detail-aware processing
- [`pointSample`](./pointSample): weighted farthest-point sampling and weighted-sampling comparison experiments built on DCI-derived descriptors

## Pipeline

The intended public workflow is:

1. Run `pdpcFeature` to extract multi-scale features and segmentation outputs.
2. Feed the exported point cloud, component, segment, variation, and scale files into `DCI`.
3. Optionally feed the point cloud together with DCI-derived `consistency` and `stability` values into `pointSample` for weighted sampling experiments.

These subprojects are versioned together because the downstream stages consume artifacts produced by the earlier stages.

## Build

Each subproject is built independently:

```bash
cmake -S pdpcFeature -B build/pdpcFeature -DCMAKE_BUILD_TYPE=Release
cmake --build build/pdpcFeature -j

cmake -S DCI -B build/DCI -DCMAKE_BUILD_TYPE=Release
cmake --build build/DCI -j

cmake -S pointSample -B build/pointSample -DCMAKE_BUILD_TYPE=Release
cmake --build build/pointSample -j
```

See the subproject READMEs for dependencies and executable usage:

- [`pdpcFeature/README.md`](./pdpcFeature/README.md)
- [`DCI/README`](./DCI/README)
- [`pointSample/README.md`](./pointSample/README.md)

## Quick Start

The lowest-friction public path is:

1. Build [`pdpcFeature`](./pdpcFeature) to generate point-cloud components, segments, variations, and scales.
2. Build [`DCI`](./DCI) with the default options to get the public `improved` executable.
3. Build [`pointSample`](./pointSample) if you also want the weighted-sampling experiments.

For a repository-local smoke test of the DCI public entry point:

```bash
cmake -S DCI -B build/DCI -DCMAKE_BUILD_TYPE=Release
cmake --build build/DCI -j
./build/DCI/src/improved DCI/examples/minimal/config.json
```

This minimal example is intentionally tiny. It validates the public config format and output path behavior, not the full paper-scale workload.

## Citation

Repository citation metadata is provided in [`CITATION.cff`](./CITATION.cff).

Current manuscript metadata:

- Title: `PointDCI: Quantifying Multi-scale Structural Significance in 3D Point Clouds`
- Authors: Ruifeng Zhao, Yubo Men, Yongmei Liu, Chaoguang Men, Jin Li, Zeyu Tian
- Venue status: Submitted to `The Visual Computer`

Until final bibliographic metadata is available, please cite the repository together with the exact release or commit you used.

The citation metadata in this repository will be updated promptly as the paper status changes.

## License Layout

This monorepo does not apply a single root license to every file.

- Files under [`pdpcFeature`](./pdpcFeature) are distributed under the license in [`pdpcFeature/LICENSE`](./pdpcFeature/LICENSE).
- Files under [`DCI`](./DCI) are distributed under the license in [`DCI/LICENSE`](./DCI/LICENSE).
- Files under [`pointSample`](./pointSample) are distributed under the license note in [`pointSample/LICENSE`](./pointSample/LICENSE), which follows the same GPL-3.0-or-later terms used by `DCI`.

Additional bundled third-party components are summarized in [`THIRD_PARTY.md`](./THIRD_PARTY.md).
Repository-level release notes are summarized in [`LICENSES.md`](./LICENSES.md).

## Status

This is research code prepared for open-source release.

- The public `DCI` build now defaults to the main `improved` executable only; research-only executables are opt-in.
- A repository-local DCI smoke example is provided under [`DCI/examples/minimal`](./DCI/examples/minimal).
- Some helper scripts remain research-oriented and are documented as such rather than treated as stable APIs.
