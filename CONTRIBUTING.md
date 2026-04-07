# Contributing

This repository is released primarily for reproducibility, inspection, and issue reporting around the paper code.

Before opening a pull request:

1. Keep changes scoped to one subproject when possible.
2. Preserve the existing file-level and directory-level license layout.
3. Update the corresponding README when changing a public command, config field, or output file.
4. Run the relevant local checks:
   - `pointSample`: build plus `ctest --test-dir build/pointSample --output-on-failure`
   - `DCI`: build the default `improved` target and run `DCI/examples/minimal/config.json`
   - `pdpcFeature`: at minimum, confirm CMake configure/build still succeeds

Issue reports are most useful when they include:

- OS and compiler version
- exact build command
- exact runtime command
- whether the problem appears in the minimal example or only on a full dataset
