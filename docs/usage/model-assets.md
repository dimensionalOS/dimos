# Model asset downloads

Use `dimos.utils.assets` for individual HTTPS and Hugging Face model files:

```python
from dimos.utils.assets import download_http_asset, download_hf_asset
from dimos.utils.cache import cache_usage_guard

# Hold the guard until your model has finished reading the returned files.
with cache_usage_guard():
    checkpoint = download_http_asset("https://example.com/model.pt")
    # Load checkpoint here.
```

`download_http_asset(url, sha256=None)` returns a local `Path`. URLs must use
HTTPS. Provide an upstream SHA-256 when available: supplied checksums are
verified on downloads and cache hits. Without a checksum, a completed cached
file is reused as-is. The cache key includes the URL and expected checksum.

`download_hf_asset(repo_id=..., revision=..., filename=...)` returns a local
`Path` at an explicit repository revision. Pin built-in models to a full commit
hash. It needs the optional `huggingface-hub` package only when called; the SDK
handles transfers and its cache layout. Cache hits do not contact the network.

Both use `dimos.constants.CACHE_DIR` (`$XDG_CACHE_HOME/dimos`, default
`~/.cache/dimos`): HTTPS files under `assets/http`, HF snapshots and blobs under
`assets/huggingface`. No helper accepts a second cache-root setting. HF model
storage uses an explicit dimOS cache path, independent of `HF_HOME`.

The existing `STATE_DIR/cache-users` markers and `STATE_DIR/cache-clean.lock`
protect download and loading operations from concurrent `dimos cache clean`.
Helpers guard their own downloads; callers must hold `cache_usage_guard()` until
finished reading returned files. Downloaded assets are disposable cache data;
user-provided files remain outside this ownership boundary. No migration of old
model caches is performed.

Concurrent HTTPS requests for the same file share a lock. Transfers write a
partial file and publish it atomically on success. Errors include the source;
retrying the operation retries a missing download. `dimos cache clean` removes
these assets after users of the cache have stopped.
