# Convert an existing recording to CDR

Conversion is an explicit offline command. It never starts a robot, downloads data,
or negotiates an old runtime wire format. Install this proposal's DimOS package;
built-in generated messages are included in it. The optional historical LCM decoder
is `dimos-lcm==0.1.4` (already retained while the runtime migration is incomplete).

Use your existing local MCAP or memory SQLite file and choose a **new** output name:

```sh skip
dimos mem convert old.db converted.mcap --dry-run
dimos mem convert old.db converted.mcap
dimos mem summary converted.mcap
dimos mem rerun converted.mcap --seconds 5 --no-gui
```

`--dry-run` checks declared types, schemas and row counts; payload integrity is
checked during conversion. It creates no output. Read the JSON report and resolve
any blocked inputs before converting. No compressed archive is downloaded or
extracted by these commands.

Either container can be an input or output:

```sh skip
dimos mem convert old.mcap converted.db
dimos mem convert old.db converted.db
```

MCAP is the portable recording format, with full dependency `ros2msg` schemas and
CDR payloads embedded. SQLite is the writable analysis/vector-search format. Vector
embeddings require SQLite output; attempting to export them to MCAP fails rather
than dropping them. Vector preservation uses the existing Embedding implementation
and its PyTorch prerequisite, loaded only if vectors are present.

An explicit directory can be migrated while preserving relative filenames:

```sh skip
dimos mem convert ./old-recordings ./cdr-recordings --format mcap --dry-run
dimos mem convert ./old-recordings ./cdr-recordings --format mcap
```

The new output directory must not overlap the source tree. Every candidate is
preflighted before any conversion. Unsupported inputs are listed as blocked.
If a payload fails after a batch has started, completed files remain; the command
returns nonzero and `migration-summary.json` records converted, failed and
not-attempted files. A batch is not an atomic transaction across files.

## Fidelity and failure behavior

- Originals are opened read-only. Existing outputs, reports and racing output
  writers are never overwritten. A single-file failure publishes no output.
- All mapped fields, exact Header seconds/nanoseconds and frame IDs are preserved.
  LCM array length fields are validated; CDR encodes the corresponding sequences.
- MCAP log time, publish time and envelope sequence retain their exact integer values.
  SQLite observation timestamps retain their original floating-point precision.
- Legacy ROS1 `Header.seq`, SQLite observation IDs/value/pose/tags and container
  metadata are retained in the mandatory adjacent `.conversion.jsonl` audit file.
  SQLite output additionally carries conversion metadata in tags, spatial poses
  and vector-to-observation associations. IDs may change and are mapped in the audit.
- An explicitly declared JPEG Image maps to standard CompressedImage without
  decoding or recompressing its bytes. JSON String retains its UTF-8 document.

The allowlist lives in `dimos/memory/convert_recording.py`. Supported standard
geometry, sensor, navigation, TF, time and scalar message shapes use the installed
historical decoder and generated CDR registry. Unknown/custom/proprietary types,
pickle, raw captures, external blob/vector stores and MCAP attachments are rejected.
No class named by a recording is imported or instantiated. Declared CDR input must
match the installed full schema. Untrusted native-format recordings are not a
sandboxed input format; use recordings from trusted producers.

The Rerun command exports generated image, cloud, pose/odometry and TF streams
through external adapters. CameraInfo is paired with its image frame; source
Header timestamps are logged at integer nanosecond precision. The generated
message classes need no viewer methods.

Open `converted.mcap` as a local file in Foxglove.
Foxglove may require your existing desktop installation or signed-in browser;
conversion does not create accounts or upload a recording. Viewer and human
acceptance remain separate from successful data conversion.

The focused regression suite runs without data downloads or hardware:

```sh
uv run pytest --noconftest --import-mode=importlib -o addopts='' -q dimos/memory/test_convert_recording.py dimos/memory/test_recording_migration.py
```
