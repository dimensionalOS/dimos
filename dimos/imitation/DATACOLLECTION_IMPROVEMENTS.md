# Data Collection Improvements

This document tracks concrete improvements to the imitation data collection pipeline, from
teleoperation inputs through Memory2 recording and dataset conversion. Priorities reflect the
risk of data loss, unsafe behavior, or silently incorrect training data.

## P0: Recording Integrity

### Use a lossless recorder queue

Memory2 currently routes recorder inputs through the asynchronous `LATEST` dispatcher. When
messages arrive faster than SQLite writes complete, pending values are replaced. This can drop
camera frames, joint states, and episode status events without reporting the loss.

- Give recorder streams a bounded FIFO instead of `LATEST` coalescing.
- Define whether queue saturation applies backpressure or rejects new messages.
- Track received, written, queued, and dropped message counts per stream.
- Emit a rate-limited warning and expose metrics whenever data is dropped.

### Drain writes during shutdown

Stopping a recorder currently cancels pending asynchronous callbacks. The store may close before
all accepted messages are durable.

- Stop accepting new messages first.
- Drain and join recorder queues.
- Commit or checkpoint SQLite.
- Close the store only after all writers finish.
- Add a timeout with an explicit error rather than silently abandoning queued data.

### Fix module shutdown ordering

Collection producers can continue publishing after the recorder begins stopping. A final episode
save or discard event can therefore be missed.

- Stop camera, teleop, coordinator, and episode producers before finalizing the recorder.
- Add lifecycle dependencies or an explicit collection finalization RPC.
- Verify that the last accepted episode marker and sensor frame are persisted.

### Define interrupted-episode behavior

An episode still recording at shutdown is currently omitted by offline extraction.

- Choose an explicit policy: persist `discard`, auto-save, or leave recoverable as incomplete.
- Default to discard unless the operator explicitly requests recovery.
- Report incomplete episodes during dataset inspection.

## P0: Teleoperation Safety

### Disengage on Quest disconnect

A disconnected Quest client can leave stale controller poses and engagement state active.

- Clear cached controller poses when the final client disconnects.
- Disengage both hands and publish an explicit safe command.
- Reject commands until fresh controller state arrives after reconnection.

### Validate controller identity

- Accept only known left and right controller frame identifiers.
- Reject unknown identifiers instead of treating them as the right controller.
- Validate button mappings at startup rather than silently ignoring invalid aliases.

## P1: Recording Performance

### Use an image-appropriate transport

Raw camera images on the default transport create substantial bandwidth and serialization load.

- Prefer shared memory for local collection pipelines.
- Evaluate `CompressedImage` for remote or cross-host recording.
- Measure throughput, CPU use, latency, and frame loss before choosing a default.
- Preserve source timestamps through compression and transport.

### Disable unused camera outputs

- Disable depth when the selected dataset configuration does not consume it.
- Avoid publishing or recording camera streams absent from the dataset schema.
- Keep TF recording disabled when all collection streams are poseless and replay does not need it.

### Batch database writes

- Append observations in bounded transactions rather than committing each message independently.
- Tune batch size and flush interval for durability and throughput.
- Force a final flush on episode boundaries and shutdown.
- Checkpoint WAL data before rotating or backing up a recording.

## P1: Synchronization Correctness

### Preserve action-shift semantics

Synchronization currently removes unmatched target times before applying `action_shift`. A shift
of one can therefore select an action multiple grid periods into the future.

- Align every feature on the target-time grid first.
- Apply action shift using target-grid indices.
- Emit a sample only when both the observation position and shifted action position are valid.
- Preserve gaps rather than compressing the timeline.

### Handle episode boundaries with tolerance

- Query source streams over `[start - tolerance, end + tolerance]`.
- Enforce the configured tolerance when selecting the nearest value.
- Prevent avoidable loss of the first and last synchronized samples.

### Validate synchronization configuration

- Require finite rates and tolerances.
- Require positive rates for fixed-rate formats.
- Require nonnegative action shifts.
- Require LeRobot FPS to match the synchronization rate.
- Reject NaN, infinite, reversed, zero-length, or overlapping explicit episode ranges.

### Order status events explicitly

- Process episode status observations in timestamp order.
- Detect and report non-monotonic or impossible transitions.
- Preserve insertion order in tests that exercise out-of-order timestamps.

## P1: Dataset Reliability

### Make output generation atomic

Writers currently operate directly on their final destinations. Failed conversion can destroy an
existing HDF5 file or leave a partial LeRobot directory that looks complete.

- Write to a sibling temporary file or directory.
- Finalize and validate the temporary dataset.
- Atomically rename it to the requested destination.
- Reject existing output by default; require an explicit overwrite option.
- Remove temporary output after failure.

### Reject empty datasets

- Fail when no successful episodes are found.
- Fail when synchronization or action shifting produces zero frames.
- Report rejected episode counts and reasons.

### Enforce one feature schema

- Establish feature names, shapes, and dtypes from the first complete sample.
- Validate every later sample before writing it.
- Reject missing, additional, ragged, or shape-changing features.
- Validate fixed image dimensions, channels, and dtype.

### Improve errors and inspection

- Include stream, feature, episode, and timestamp context in conversion errors.
- Validate actual row counts against metadata totals.
- Validate exact feature sets, shapes, dtypes, video files, and video frame counts.
- Do not report shape uniformity without checking it.

## P1: LeRobot Integration

The current writer manually implements the LeRobot v3 layout. The official
`LeRobotDataset.create()`, `add_frame()`, `save_episode()`, and `finalize()` API could replace
most custom Parquet, video, task, statistics, and sharding code.

### Preferred integration pattern

- Add LeRobot as a dedicated optional dependency, separate from lightweight HDF5 support.
- Pin one tested LeRobot release.
- Keep imports lazy so DimOS remains importable without the optional runtime.
- Use dependency overrides only after validating the chosen versions across DimOS.
- Inspect the universal lock for unexpected changes to Torch, NumPy, OpenCV, Rerun, PyAV,
  Transformers, and `huggingface-hub`.
- Keep LeRobot outside the `all` extra until supported platforms pass compatibility testing.

### Thin writer adapter

- Infer the complete feature schema from the first sample.
- Map one low-dimensional observation to `observation.state`.
- Map one action to `action`.
- Map image observations to `observation.images.<name>`.
- Save an episode whenever `episode_id` changes.
- Finalize the official writer on success and after recoverable partial failure.
- Reopen generated output with the official LeRobot loader in tests.

### Compatibility questions

- Determine whether LeRobot 0.4.x can run safely with DimOS dependency overrides.
- Avoid installing both `opencv-python-headless` and `opencv-contrib-python`; both provide `cv2`.
- Verify whether LeRobot's Rerun, Hub, and PyAV bounds are required by dataset writing or are stale
  package-wide constraints.
- Do not override a genuine runtime incompatibility only to satisfy the resolver.

## P2: Memory Efficiency

- Query and decode each unique source stream once per episode.
- Use rolling cursors for nearest-sample alignment instead of repeated binary searches.
- Retain only the `action_shift + 1` frame window needed for shifted actions.
- Stream HDF5 output into resizable, chunked datasets instead of buffering whole episodes.
- Avoid stacking a second full copy of large image episodes in memory.

## P2: Training Data Semantics

### Record the commanded action

The current example derives actions by shifting measured joint state. This represents next
measured state, not necessarily the command selected by the controller after arbitration,
clamping, or hardware lag.

- Record the post-arbitration command target as a separate stream.
- Preserve measured state as observation.
- Name shifted measured-state targets explicitly if that training mode remains supported.

### Correct image statistics

- Compute channel-wise pixel sums, squared sums, minima, maxima, and counts.
- Do not calculate image variance from per-frame channel means.
- Keep memory usage independent of image resolution.
- Match official LeRobot normalization scale and quantile semantics when producing LeRobot data.

## P2: Storage Semantics

- Make `OnExisting.APPEND` truly append, or rename it to indicate stream replacement.
- Rotate SQLite databases together with `-wal` and `-shm` sidecars.
- Checkpoint WAL before backup or overwrite.
- Fall back from message timestamps only when the value is `None`; preserve a valid `0.0`.

## Test Plan

### Recorder tests

- Burst traffic writes every accepted message in order.
- Queue saturation is observable and follows the configured policy.
- Shutdown drains pending messages.
- Store closure happens after writer completion.
- Final episode status survives shutdown.

### End-to-end collection test

- Publish synthetic image, joint-state, and status streams.
- Record them into a real temporary SQLite store.
- Extract saved and discarded episodes.
- Synchronize observation and action features.
- Write HDF5 and LeRobot outputs.
- Reopen each output and validate values, timestamps, episode boundaries, and counts.

### Dataset failure tests

- Empty synchronized output fails clearly.
- Schema drift fails before partial final output is published.
- Existing output is not destroyed without explicit overwrite.
- Mid-conversion exceptions remove temporary artifacts.
- Multi-shard LeRobot metadata is fully inspected.

## Suggested Phases

1. Make recorder queuing lossless and shutdown drain-safe.
2. Fix Quest disconnect safety and interrupted-episode behavior.
3. Add recorder metrics and the SQLite end-to-end test.
4. Reduce image transport and unused depth/TF work.
5. Correct synchronization and action-shift semantics.
6. Make dataset output atomic and schema-validated.
7. Evaluate the official LeRobot writer against a pinned, validated dependency set.
8. Improve memory use, statistics, and commanded-action semantics.
