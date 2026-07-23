# Store data assets on Hugging Face

DimOS stores one `tar.gz` archive per top-level data asset under `data/` in the
public `playercc7/dimensional` Hugging Face dataset and pins one full dataset
commit SHA centrally. Each `data/<asset>.tar.gz` archive contains exactly one
`<asset>` root. `get_data()` downloads the archive lazily, validates it, and
extracts it locally into the established `data/` tree; the remote files remain
archives. This preserves `get_data()` and `LfsPath` as storage-independent
references while avoiding excessive per-file Hub requests for recordings with
many small files.
