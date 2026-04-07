# License Guide

`pointDCI` is a mixed-license research monorepo.

The effective license depends on which subdirectory you use:

- [`pdpcFeature`](./pdpcFeature) is distributed under [`MPL-2.0`](./pdpcFeature/LICENSE).
- [`DCI`](./DCI) is distributed under [`GPL-3.0-or-later`](./DCI/LICENSE).
- [`pointSample`](./pointSample) follows the same [`GPL-3.0-or-later`](./DCI/LICENSE) terms as `DCI`, with a directory-local notice in [`pointSample/LICENSE`](./pointSample/LICENSE).

Bundled third-party code and provenance notes are summarized in [`THIRD_PARTY.md`](./THIRD_PARTY.md).

Practical implications:

- Reusing only `pdpcFeature` is governed by MPL-2.0 for that directory.
- Reusing `DCI` or `pointSample` is governed by GPL-3.0-or-later for those directories.
- Reusing the monorepo as a combined distribution requires respecting the most restrictive applicable terms for the files you redistribute.

This file is a release aid for repository users. The license texts in each subproject directory remain the authoritative source.
