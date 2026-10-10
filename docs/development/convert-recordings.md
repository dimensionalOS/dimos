# Convert an old recording to CDR

This offline tool is for reviewing historical dimOS recordings on the CDR proposal.
It does not start a blueprint, router, robot, viewer or network connection.

From this proposal checkout, use an environment containing dimOS, its matching
`dimos-generated` package and the retained historical decoder. The commands below
require your existing historical recording: replace `old.mcap` or `old.db` with
its path. These input files are not created by the walkthrough.

```sh skip
pip install dimos-lcm==0.1.4
dimos mem convert old.mcap converted.mcap
```

Open `converted.mcap` directly in Foxglove with **Open local file**. Select your
image topic in an Image panel and point cloud / pose topics in a 3D panel. A local
Foxglove application or an existing signed-in web session may be needed; the
converter does not create an account or upload your recording.

For an old memory SQLite recording, or for CDR SQLite output:

```sh skip
dimos mem convert old.db converted.mcap
dimos mem convert old.mcap converted.db
dimos mem convert old.db converted.db
```

Choose a **new output filename**. The original file is opened read-only. Existing
outputs and reports are refused, including a destination created concurrently.
Conversion writes to a temporary sibling directory; no output is published if
any stream or payload fails. Output directories must already exist.

Inspect either output through the current CDR memory reader:

```sh skip
dimos mem summary converted.mcap
dimos mem summary converted.db
```

The converter loads the allowlisted historical decoders from `dimos-lcm`.
Historical `dimos.msgs` names are recognized as input metadata; their old runtime
classes and `lcm_encode`/`lcm_decode` APIs are not restored. This reuses the original
decoder implementation; it introduces no new codec.
New runtime transports and recordings still use generated CDR types; there is
no automatic old-wire fallback.
For JSON-only input it is unnecessary. `mcap`/`lz4` and SQLite support come from
the prepared dimOS environment. No dependencies are downloaded by conversion.

## Accepted input and mapping

| Input | Output |
| --- | --- |
| dimOS MCAP `lcm`, `jpeg`, `json`, optionally `lz4+…` | Generated standard ROS-compatible CDR values |
| Memory SQLite `_streams` registry, in-file observation/blob tables, same codecs | Same CDR values through the actual `SqliteStore` |
| Known CDR MCAP with `ros2msg` exactly matching the installed schema | Match the schema and decode/re-encode without changing the type |
| LCM Image with explicit `jpeg` codec | `sensor_msgs/msg/CompressedImage`, `format="jpeg"`, **original JPEG bytes** |
| Raw LCM Image / PointCloud2 / Imu / PoseStamped / Odometry / TFMessage and listed standard messages | Same ROS message shape; `std_msgs.Time.nsec` becomes `builtin_interfaces/Time.nanosec` |
| Explicit JSON `std_msgs.String` | Same UTF-8 JSON document inside generated `std_msgs/msg/String` |

The full, explicit allowlist is `TYPES` in
`dimos/memory/convert_recording.py`. Only standard messages with compatible field
semantics are admitted; unknown fields/types are errors. LCM length fields are
checked against their arrays, then represented by CDR sequence lengths. The
installed historical decoder checks the LCM fingerprint.

CDR input must come from trusted producers and match the installed schema. The
native Python decoder is not a hostile-input validation boundary: see the
[accepted native-library limitations](/docs/development/message-limitations.md).
A matching schema does not make malformed bytes safe. Conversion uses the shared
registry codecs and native value types, with no fallback to the old message API.

MCAP reads `dimos.payload_type` metadata. Historical
`dimos/<stream>/<Type>` wire names are also recognized when the type is uniquely
listed. It never imports a Python class or SQLite component named by an input
file. **Pickle is never loaded.** Unknown streams are listed together before an
output is created; detected payload errors also leave no published output.

Go2 DDS recordings use different conventions (`rt/utlidar/cloud`,
`rt/frontvideo`, proprietary Unitree types). A topic name alone does not specify
its bytes. Standard CDR channels need their matching full `ros2msg` definition;
undeclared raw video, proprietary DDS types and old Go2 pickle directories are
rejected rather than guessed. They need a separately reviewed, explicit export
from their trusted original producer.

## What is preserved

- Stream/topic names and per-stream message count/order, including empty streams.
  SQLite requires SQL identifier stream names; use MCAP for names containing `/`.
- All mapped message fields, including Header frame and exact sec/nanosec values.
- MCAP input's exact integer `log_time`, `publish_time` and envelope `sequence`.
  They are **not** replaced with conversion wall time or guessed Header time.
- SQLite's existing floating-point observation time. For MCAP output it supplies
  log time (rounded to nanoseconds); Header supplies publish time when present.
  Precision absent from the source float cannot be recovered.
- SQLite observation ID, value, pose and tags, and legacy ROS1 `Header.seq`, in a
  mandatory adjacent `converted.mcap.conversion.jsonl` / `.db.conversion.jsonl`
  audit file. MCAP container metadata is retained in the audit manifest. SQLite output additionally carries these in `cdr_conversion` tags
  and retains the spatial pose. ROS2 Header has no `seq`; for SQLite input its
  legacy value also supplies the MCAP sequence. For MCAP input the envelope
  sequence wins and the original Header sequence remains in the audit file.

MCAP uses `message_encoding="cdr"`, XCDR1 little endian,
`schema_encoding="ros2msg"`, full nested definitions, the `ros2` profile and ZSTD
chunks. It is not LCM bytes relabeled as CDR. SQLite output uses the current
`CdrCodec` and real store registry/blob format. Runtime import paths in output
refer to generated classes.

This migrates message recordings, not arbitrary Python object databases. External
blob stores, MCAP attachments and pickle payloads are rejected instead of
silently discarded. Keep the audit file with the converted recording.

## Validation

With the historical decoder installed:

```sh skip
python -m pytest dimos/memory/test_convert_recording.py dimos/protocol/test_cdr_mcap.py
```

Tests cover both containers, LZ4/JSON/JPEG/LCM, independent rosbags decoding,
Header/type mapping, timestamp/sequence retention, unsupported types, corrupt
payloads, empty streams and exclusive output publication. Legacy-specific tests
are skipped if the historical decoder is missing; that is not a full
converter acceptance run.

## Convert a directory explicitly

```sh skip
dimos mem convert ./recordings ./recordings-cdr --dry-run
dimos mem convert ./recordings ./recordings-cdr
dimos mem convert ./recordings ./recordings-cdr-sqlite --format db
```

The destination must be new, with an existing parent, and must not overlap the
source tree. Relative directories are preserved: `trip/run.db` becomes
`trip/run.db.cdr.mcap`. No home-directory scan or automatic download occurs.
Dry-run reads schema declarations and counts rows; it does not prove payload
integrity. Both dry-run and conversion exit nonzero when candidates are blocked.
Archives, LFS pointers, pickle, raw captures and symlinks are reported as blocked;
unrelated non-recording files are ignored. Extract/materialize trusted archives
separately before selecting an expanded source directory.

All candidates pass metadata preflight before any conversion starts. Each output
and its audit file are published exclusively. If a later payload fails, earlier
successful files remain, later files are marked not attempted, and the command
fails. `migration-summary.json` records every candidate and result. Retain the
originals; a successful conversion is not authorization to delete them.

## Recording versus writable analysis

MCAP is a cheap bus-recording format, not an embedding database. Convert a
recording into the standard writable SQLite backend before analysis:

```sh skip
dimos mem convert converted.mcap analysis.db
dimos mem summary analysis.db
```

Both filenames must be new when used as outputs. This uses the existing
`SqliteStore`, not a new database format. MCAP integer timestamps and sequence
are retained in `cdr_conversion` observation tags and the mandatory audit file;
SQLite observation timestamps remain floating point.

For historical SQLite recordings with embeddings, use SQLite output. This path
uses the existing `Embedding` class and requires its PyTorch dependency (already
present in the development environment); it does not load a model or weights:

```sh skip
uv pip install torch  # only if absent from a core-only environment
dimos mem convert go2_short.db go2_short.cdr.db --dry-run &&
dimos mem convert go2_short.db go2_short.cdr.db
```

In-file sqlite-vec float32/cosine vectors are copied through the existing vector
store, linked to newly assigned observation IDs. The report records the old/new
IDs. Payloads, poses and original tags remain associated with the observation.
External/custom vector stores, unsupported vector schemas and orphaned vectors
are rejected. DB-to-MCAP with vectors remains an error; there is no discard flag,
embedding channel, attachment or automatic sidecar-vector export.

Existing Python APIs remain `Stream.append(data, embedding=embedding)`,
`stream.transform(EmbedImages(model)).save(destination).drain()` and
`stream.search(query_embedding, k=10)`. The first two append new observations;
they do not attach embeddings to an existing row. Search operates through the
SQLite vector store. There is no embedding-analysis CLI and no MCAP vector search.
Use the same model/preprocessing for queries as for stored vectors: historical
vectors do not establish model identity merely from their dimension. Compressed
images must be decoded to the input type expected by the chosen embedding model.
No model weights are downloaded or embeddings recomputed by conversion. Realtime
capture plus embedding analysis should write to the normal SQLite backend.

Validation on Linux: the complete `go2_short.db` migration retained 2,546
observations and all 108 stored 512-dimensional vectors, with exact vector bytes
and verified old/new ID associations. Real stairs MCAP-to-SQLite conversion
retained all 4,451 payloads and exact envelope metadata. A focused regression
checks noncontiguous source IDs and ranked search after migration. This does not
claim newly validated model inference, macOS execution or MCAP embedding support.

## Ivan's offline acceptance checklist

Run from this proposal checkout with its prepared development environment activated
(`source .venv/bin/activate`). It must contain matching `dimos-generated`, MCAP,
NumPy, Open3D and Rerun packages. These commands do not build/install dependencies
or start a robot. Use a fresh directory for each run:

```sh skip
export CDR_REVIEW_DIR="$(mktemp -d "${TMPDIR:-/tmp}/dimos-cdr-review.XXXXXX")"
```

### Full real legacy point-cloud recording

Use `go2_mid360_stairs.db.tar.gz`, an existing approximately 60-second legacy
recording: 54,082,131 compressed bytes and 155,848,704 extracted bytes. It contains
LCM point clouds/poses and JPEG images. This is a direct SQLite-to-CDR conversion;
no Python packaging script or synthetic fixture is needed. Download only this
archive, verify its project LFS hash, and extract into the fresh review directory:

```sh skip
git lfs pull --include="data/.lfs/go2_mid360_stairs.db.tar.gz" --exclude=""
printf '%s\n' '02d8d2194332291cf71988458965aa9f8019b8d61661ff153e93886dd72e85e9  data/.lfs/go2_mid360_stairs.db.tar.gz' | shasum -a 256 -c -
tar -xzf data/.lfs/go2_mid360_stairs.db.tar.gz -C "$CDR_REVIEW_DIR"
dimos mem convert "$CDR_REVIEW_DIR/go2_mid360_stairs.db" "$CDR_REVIEW_DIR/converted.mcap" --dry-run &&
dimos mem convert "$CDR_REVIEW_DIR/go2_mid360_stairs.db" "$CDR_REVIEW_DIR/converted.mcap"
dimos mem summary "$CDR_REVIEW_DIR/converted.mcap"
```

Expect 4,451 messages: `color_image` 855, `fastlio_lidar` 576,
`fastlio_odometry` 1,595, `lidar` 301 and `odom` 1,124. Conversion retains every
stream. Images become `sensor_msgs/msg/CompressedImage`, retaining the original
JPEG bytes. Keep the original SQLite file and adjacent conversion audit report.

`go2_short.db` includes a `color_image_embedded` vector index: migrate it to
`.db`, not `.mcap`, to preserve embeddings. MCAP output rejects vectors explicitly. `alfred_fusion_short.db` converts but
has no point clouds or images. Its current Rerun renderer logs only TF/odometry
transforms, without explicit axes or visible geometry, and skips IMU; a populated
entity tree therefore does not guarantee a visible 3D scene.

### Memory rendering, mapping and replay

For a core-only checkout environment, mapping currently also imports Unitree
helpers and Matplotlib. Install the declared Unitree extra and the plotting
package if absent (the full development environment already has these):

```sh skip
uv sync --frozen --python 3.12 --no-default-groups --extra unitree
source .venv/bin/activate
uv pip install 'mcap>=1.2.0' 'dimos-lcm==0.1.4' 'matplotlib>=3.7.1'
```

The following bound visualization to five seconds and use CPU mapping:

```sh skip
dimos mem rerun "$CDR_REVIEW_DIR/converted.mcap" --seconds 5 --no-gui --out "$CDR_REVIEW_DIR/memory.rrd"
dimos map global "$CDR_REVIEW_DIR/converted.mcap" --lidar lidar --duration 5 --device CPU:0 --block-count 10000 --no-gui --out "$CDR_REVIEW_DIR/global.rrd"
dimos map replay "$CDR_REVIEW_DIR/converted.mcap" --lidar lidar --duration 5 --map-final --map-device CPU:0 --no-gui --out "$CDR_REVIEW_DIR/replay.rrd"
```

There is no `dimos mem replay` command. `mem rerun` renders recorded messages,
including compressed images. `map replay` currently selects only uncompressed
`Image` streams, so this recording's JPEG images are absent from its output;
do not pass `--image color_image`. Use `memory.rrd` for images plus clouds and
`global.rrd` for the accumulated map. Neither command starts live blueprint
transport replay.

`--lidar lidar` is necessary because this recording has two compatible cloud
streams. Automatic selection works only with a unique candidate. Its world-frame
clouds need no stored `obs.pose`; spatial dedup defaults off (`--pgo-tol 0`).
Explicit dedup/PGO requires trajectory metadata. Separate recorded odometry is
not automatically attached as `obs.pose`. Sensor-frame clouds need recorded TF
registration. Missing required data is an error, not an invented pose.

`map global --markers` and `map replay-marker` additionally need usable camera
calibration, supported image types and poses; this checklist does not claim
marker acceptance. `map rename` and `map pose-fill` remain SQLite-only.

### Independent validation and manual viewers

If the separately installed official Foxglove MCAP CLI is on PATH:

```sh skip
mcap doctor "$CDR_REVIEW_DIR/converted.mcap"
```

The Python `mcap` package is not that executable. No installation is performed by
these commands. Doctor has not been run for this acceptance; CRC/payload checks
are not a substitute for it.

Open the generated files locally (manual visual acceptance):

```sh skip
rerun --serve-web --bind 127.0.0.1 --web-viewer-port 9090 "$CDR_REVIEW_DIR/memory.rrd"
```

Open http://127.0.0.1:9090 and select timeline `time`. Stop this process with
Ctrl-C before starting another viewer on the same port. Substitute `global.rrd`
for the accumulated map (`/world/raw_map/pointcloud`) or `replay.rrd` for animated
clouds/trajectories (timeline `ts`). Add a 3D view containing the relevant entity
if the viewer does not create it automatically.

In Foxglove, **Open local file** `converted.mcap`; select `lidar` in a 3D panel
with fixed frame `world`, `color_image` in an Image panel, and inspect `odom` in
Raw Messages. Do not assume camera/odometry overlays align without a matching TF
chain. Existing application/session access may be required; do not upload private
recordings to obtain acceptance. Successful RRD export does not constitute human
visual review.

### Focused regression entry

```sh skip
python -m pytest dimos/memory/test_convert_recording.py dimos/protocol/test_cdr_mcap.py dimos/memory/store/test_mcap.py dimos/mapping/cli/test_stream_selection.py dimos/mapping/cli/test_pgo_accumulate.py dimos/mapping/loop_closure/test_pgo.py
```

At published `6256c71becff76f9ae60af0b4076a145941bdf98`, the actual stairs
archive passed hash verification, registry preflight and full conversion of all
4,451 legacy messages. Linux CPU checks rendered 70 images in the five-second
memory view, retained 39/39 clouds in global mapping and completed map replay.
These are real legacy recording checks, not repackaged CDR fixtures. The runtime
layer separately passed 40 focused mapping/reader/PGO tests and the converter
layer 34 converter/migration/writer tests. macOS installation/execution, doctor,
human visual review, full archive migration and full-suite CI are not claimed.

The 2026-10-09 native-library integration was also checked locally using the
existing real fixtures: all 4,451 stairs messages matched independently decoded
LCM fields and exact output envelope times; all 2,546 Go2-short observations and
108 stored 512-dimensional vectors were retained, including exact vector bytes
and old/new observation-ID associations. Both source database hashes remained
unchanged. The converter/writer regression suite passed 38 tests, including
nonempty numeric and byte arrays. These results do not establish strict malformed
CDR rejection; the accepted native-library limitations above still apply.

## Built-in region streams

The converter recognizes the old mapper/planner contracts by exact stream name
and source type: `seed_map`, `map_regions`, `surface_map` (PointCloud2),
`seed_bounds`, `region_bounds` (PoseStamped), and `node_edges` (Path).
Historical `dimos/<name>/<type>` wire topics are also recognized. Other streams
retain their ordinary type; a matching basename under an arbitrary external topic
is not sufficient to infer region semantics.

For these legacy LCM streams, signed `Header.seq` becomes `region_id` unchanged.
Clouds remain standard PointCloud2 values inside their region envelope. Bounds
become explicit center/radius/vertical limits. Edge pose pairs become independent
start/end/weight segments; inconsistent frames, timestamps or pair weight layouts
fail conversion rather than losing fields. Endpoint Header sequence values remain
in the conversion sidecar. Empty clouds and edge arrays retain their region ID.

MCAP's separate unsigned sequence retains the input envelope sequence. When only
a signed legacy Header sequence is available, its 32-bit bit pattern supplies that
MCAP field; the original signed value is retained in the sidecar and region ID.
Typed legacy recordings still require explicit conversion; runtime does not decode
them transparently.
