---
title: "Data Loading"
---

DimOS resolves data references through `get_data()` and `LfsPath`. The public
read API is unchanged, but the backing store is the public Hugging Face
dataset [`playercc7/dimensional`](https://huggingface.co/datasets/playercc7/dimensional).
The remote directory is rooted at `data/` and contains one archive per
top-level asset (`data/<top-level>.tar.gz`). Runtime reads use the immutable
dataset revision
`f262d7f8775c2d507b1bfde62a5aa21cffabb3a1`.

```python skip
from dimos.utils.data import get_data

image = get_data("osm_map_test/full.png")
```

## How It Works

Reads are local-first. If `data/{reference}` already exists, no network call is
made. Otherwise the shim downloads only `data/<top-level>.tar.gz`, validates
the archive's single root and safe members, and extracts it into
same-filesystem staging beneath the established DimOS data directory. For
cooperating writers, only the initial staged-root rename/install is atomic.
Top-level assets are immutable at the pinned revision: an existing local root
is authoritative and is never replaced. A missing nested member beneath an
existing root raises `FileNotFoundError` without a download. Paths into the
Hugging Face cache are never exposed, and unrelated archives are never
downloaded.

All in-repository top-level writers use the same per-asset advisory `filelock`,
including relative `LegacyPickleStore` recording writes. Cooperative callers
wait for the active materialization rather than failing because a legitimate
large archive takes time to download or extract. Writers that do not acquire
the lock are external/non-participating writers and are unsupported; the lock
does not claim to protect against their races. Lock files may remain after
release and are not state markers.

`LfsPath("assets/example.bin")` keeps the same lazy behavior: construction is
local, and the first filesystem operation resolves the reference. An existing
absolute local path passed to `get_data()` is returned unchanged; a missing
absolute path raises `FileNotFoundError`. Relative traversal or unsafe
references raise `ValueError`. Hub metadata, network, cache, and operational
failures are reported as `RuntimeError`; a genuinely absent remote asset is
`FileNotFoundError`.

## Common Patterns

### Loading Point Clouds

```python skip
from dimos.utils.data import get_data
from dimos.mapping.pointclouds.util import read_pointcloud

pointcloud = read_pointcloud(get_data("apartment") / "sum.ply")
print(f"Loaded pointcloud with {len(pointcloud.points)} points")
```

```results
Loaded pointcloud with 63672 points
```

## Data Directory Structure

Materialized data files live under `data/` at the repo root (or the installed
package's synthetic data root). The Hugging Face dataset stores archives, while
`get_data()` transparently materializes their top-level roots locally:

```text
remote data/
  osm_map_test.tar.gz
  apartment.tar.gz

local data/
  osm_map_test/
    full.png
  apartment/
    sum.ply
```

## Publishing Data

Publication is managed outside this repository through the Hugging Face
dataset. The runtime does not use a branch, environment override, mutable main
fallback, or replacement/rollback generation mechanism. Source-repository Git
LFS is not part of runtime data loading.

## Location Resolution

When running from:

- **Git repo**: Uses `{repo}/data/`.
- **Installed package**: Uses the established synthetic project root
  `<user-data>/repo/data`:
  - Linux: `~/.local/share/dimos/repo/data/`
  - macOS: `~/Library/Application Support/dimos/repo/data/`
  - Platform fallback: `/tmp/dimos/repo/data/`
  - Virtual environment: `site-packages/dimos/data` when `VIRTUAL_ENV` is set.

The runtime does not clone the source repository or invoke Git LFS.

## Docs media assets

Binary media displayed by the docs (screenshots, plots, GIFs) remains
LFS-tracked here under `docs/**/assets/`, but Mintlify deploys the docs site
without fetching Git LFS.

Content behind those LFS pointers is hosted as plain git blobs in
[dimensionalOS/dimos-docs-assets](https://github.com/dimensionalOS/dimos-docs-assets),
mirroring the `docs` tree minus the `docs/` prefix. Docs pages reference that
copy with absolute URLs:

```markdown
![Coverage](https://raw.githubusercontent.com/dimensionalOS/dimos-docs-assets/main/capabilities/navigation/assets/coverage.png)
```

Small text SVGs emitted by pikchr and `to_svg` are exempt: they are committed
here as plain text and referenced relatively.
