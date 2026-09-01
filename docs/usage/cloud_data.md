# Cloud data

`dimos data` uploads recordings, or any file, to Dimensional's hosted storage and pulls them back byte-identical on any machine. Authenticate once with `dimos login` (device code; the key lands in the system keyring, or `~/.config/dimos/credentials`).

```bash
dimos login
dimos data upload                                  # newest recording under recordings/
dimos data upload latest                           # same, spelled out
dimos data upload recordings/<run-id>/memory.db    # a specific file
dimos data upload --since 1h                       # every recording from the last hour
dimos data ls
dimos data pull                                    # newest upload -> downloads/
dimos data pull 9a6bf9ab                           # id prefix from ls
dimos data pull latest --dest my.db
dimos data quota
```

## Uploads

Any file type is accepted. Kind is inferred: a mem2 SQLite store (has a `_streams` table) or `.mcap` is a `recording`, anything else a `blob`. Override it with `--kind`. `--robot` tags a robot id, `--chunk` sets the multipart part size in MB.

- Files are compressed before transfer (lz4 by default, see below), sha256-verified server-side, and deduplicated by content: re-uploading the same bytes reports `already uploaded`.
- Interrupted uploads resume: the server's part listing is the only resume state, so a killed upload re-sends only the missing parts.
- Recordings carry a manifest: the stream list from `_streams` plus the blueprint name parsed from the `<stamp>-<blueprint>` run directory. `dimos data ls` shows both.
- Discovery modes (no argument, `--since`) skip files modified within the last `dimos_upload_quiet_s` seconds (default 30) so a store that is still being written is not shipped mid-run. Naming a path or `latest` is explicit intent and uploads immediately.
- Compression stages next to the source file (not `/tmp`), with a free-space check first; point `dimos_staging_dir` at a bigger partition if needed.

## Pulls

Pulls land in `downloads/` under the checkout (`~/.local/state/dimos/downloads/` for an installed package), named `<upload-time>-<id-prefix>-<filename>` so nothing overwrites. `--dest` takes an exact target path. The wire bytes are sha256-verified, decoded, and moved into place atomically; a failed pull never clobbers an existing destination file.

## Compression

`dimos_upload_codec` selects the algorithm; the upload's `content_encoding` stamp selects the decoder on pull, so mixed-codec stores coexist.

| codec | ratio* | compress* | notes |
|-------|--------|-----------|-------|
| `lz4` | 0.59 | 0.04s | default; near-free CPU |
| `gzip` | 0.49 | 1.6s | |
| `bz2` | 0.46 | 0.5s | |
| `xz` | 0.40 | 2.9s | densest, slowest |
| `""` | 1.00 | 0s | upload raw |

*measured on a 12.6 MB Go2 recording; ratios vary with content.

Like every `GlobalConfig` field, it is settable three ways (global flags go before the subcommand):

```bash
dimos --dimos-upload-codec xz data upload
DIMOS_UPLOAD_CODEC=gzip dimos data upload
echo 'DIMOS_UPLOAD_CODEC=gzip' >> .env
```

## Configuration

| Field | Default | Meaning |
|-------|---------|---------|
| `dimos_cloud_url` | `https://api.dimensional.org` | API host |
| `dimos_api_key` | `None` | Overrides the stored `dimos login` credential (`DIMOS_API_KEY` for CI) |
| `dimos_upload_codec` | `lz4` | `lz4`, `gzip`, `bz2`, `xz`, or `""` for no compression |
| `dimos_upload_chunk_mb` | `None` | Multipart part size in MB (`--chunk`); server default 64 |
| `dimos_upload_retries` | `2` | Per-part retries for transfers |
| `dimos_upload_quiet_s` | `30.0` | Discovery skips files modified this recently |
| `dimos_http_timeout` | `60.0` | Seconds per API request |
| `dimos_staging_dir` | `None` | Compression/decode staging dir; default is beside the file |

Implementation: [`dimos/cloud/data.py`](/dimos/cloud/data.py); codec table in [`dimos/constants.py`](/dimos/constants.py).
