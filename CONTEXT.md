# Data Assets

DimOS uses versioned data assets for examples, tests, models, and robot replays that are too large to distribute in the source repository.

## Language

**Data asset**:
A file or directory addressed relative to the DimOS data tree and loaded through the stable data-reference API.
_Avoid_: LFS object, archive

**Data tree**:
The unpacked hierarchy of data assets, rooted at `data/` locally. The canonical dataset stores one validated archive per top-level asset under `data/`.
_Avoid_: LFS directory, archive store

**Data reference**:
A relative path passed to `get_data()` or represented by `LfsPath`; its meaning is independent of the storage backend.
_Avoid_: LFS path, download URL

**Data revision**:
The immutable dataset commit containing the canonical top-level asset archives.
_Avoid_: branch, latest version
