# Isolated LeRobot export

LeRobot dataset conversion and inspection run in the existing isolated project
at [native/python/lerobot](/native/python/lerobot/pyproject.toml), alongside the
policy runtime. The host does not maintain its own LeRobot writer or reader.
HDF5 output continues to use the host writer.

## Host and runtime boundary

The host API in [adapter.py](/dimos/imitation/dataprep/formats/lerobot/adapter.py) provides
`run_lerobot_dataprep(config)` and `inspect_lerobot_dataset(path)`. Input and
output paths become absolute before entering the child process, whose working
directory is the isolated project.

The existing isolated-Python launcher runs `python -m dimos_lerobot.dataprep`
through `uv run --frozen --with-editable` using this source checkout. It removes
host `VIRTUAL_ENV`, `UV_PYTHON`, and `UV_PROJECT_ENVIRONMENT` pins and selects a
durable cached environment for the project. The project selects Python 3.12,
LeRobot 0.6.0, and its locked dataset/video dependencies. The first invocation
may need to provision those dependencies; a host-side LeRobot installation is
not a substitute for that environment.

The private subprocess protocol uses Pydantic build/inspect request and result
models from [protocol.py](/dimos/imitation/dataprep/formats/lerobot/protocol.py).
One JSON request is sent on stdin. A discriminated `command` identifies the
request; the result must have the matching command. Diagnostic output goes to
stderr, leaving the result on stdout. Missing `uv`, a nonzero child exit status,
and an invalid or mismatched result become host exceptions with diagnostics.
This protocol is a local process boundary, independent of recorder transports.

This executable example validates a build request without starting the runtime:

```python no-result
from pathlib import Path

from dimos.imitation.dataprep.formats.lerobot.protocol import BuildRequest, REQUEST_ADAPTER
from dimos.imitation.dataprep.core import DataPrepConfig

config = DataPrepConfig.model_validate_json(
    Path("dimos/imitation/dataprep/example_config.json").read_text()
)
request = BuildRequest(config=config)
assert REQUEST_ADAPTER.validate_json(request.model_dump_json()) == request
assert config.output.metadata["repo_id"] == "dimos/example"
```

## Dataset contract

The child reuses the host's [feature and alignment contract](/dimos/imitation/dataprep/README.md), then
writes through LeRobot's `LeRobotDataset.create`, `add_frame`, `save_episode`,
and `finalize` APIs. It does not infer feature dimensions from arbitrary data
or substitute measured state for accepted-command targets.

LeRobot output requires:

- A nonempty `output.metadata.repo_id`. This is a dataset identity; export does
  not call an upload API.
- A positive integer frame rate. `run_dataprep` derives `fps` from
  `sync.rate_hz` unless output metadata explicitly supplies it. Use the dataset
  rate for both so the exported timeline matches alignment.
- An explicit feature schema. `run_dataprep` supplies it from the configured
  features, including `complementary_info.is_filled`.
- A nonempty task label on every frame. Saved episode labels take precedence.
  Explicit ranges have no recorded label, so they use
  `output.metadata.default_task_label` (default `"task"`).
- Stable feature keys, dimensions and dtypes throughout each episode, and
  contiguous frames for each episode.

Use standard LeRobot feature names such as `observation.state`,
`observation.images.wrist` and `action` when preparing data for a policy with
that contract. The generic sample config's names are not a promise of
compatibility with every checkpoint.

`output.metadata.robot_type` names the robot in LeRobot metadata; the existing
`output.metadata.robot` key remains supported when `robot_type` is absent.

Exports are built in a temporary sibling directory and published only after all
episodes and finalization succeed. A failed export leaves no dataset at a new
output path and preserves a previous dataset during a rebuild. Rebuilding a
recognized LeRobot dataset or an empty directory replaces it after success;
unrelated directories, files and symbolic links are rejected. Use an output
path reserved for this export, and avoid concurrent builds to the same path.

Episode boundaries are committed through `save_episode` with synchronous video
encoding, followed by dataset finalization. The official dataset APIs own the
Parquet, video, task and metadata layout. The DimOS sidecar records the source,
schemas, alignment settings and quality reports. Inspection uses
`LeRobotDatasetMetadata` to report dataset features and episode/frame counts.

## Entry points

For a `DataPrepConfig` with `output.format="lerobot"`, call
`run_lerobot_dataprep(config)`; use `inspect_lerobot_dataset(path)` for its output.
Calling the generic host `run_dataprep(config)` with that format raises an error
directing you to the isolated entry point. For HDF5, use the generic host API.

At this layer of the stack, `dimos dataprep build --config CONFIG --source
RECORDING --output DATASET --format lerobot` dispatches through the same isolated
API. Its inspection adapter also dispatches LeRobot directories into the child;
SQLite/MCAP source and HDF5 inspection stay on the host. The old command remains
available until the atomic `dimos imitation` namespace migration adds its
replacement and removes the old registration together.

Only prepare trusted recordings. The isolated reader needs the Python classes
for custom typed MCAP channels. Copying a dataset does not copy its source
recording or install those classes.

## Testing this layer

Host protocol and dispatch tests run without installing LeRobot on the host:

```bash skip
uv run pytest dimos/imitation/dataprep/test_lerobot.py
```

The native writer and recording-to-dataset round trip require the isolated
project. Run both explicitly; the root `dimos` test discovery does not collect
these files:

```bash skip
cd native/python/lerobot
env -u VIRTUAL_ENV -u UV_PYTHON -u UV_PROJECT_ENVIRONMENT UV_NO_SYNC=0 \
  uv run --frozen --group tests --with-editable ../../.. pytest \
  dimos_lerobot/dataprep_tests.py dimos_lerobot/collection_dataprep_tests.py
```

CI runs these two native test files in `isolated-lerobot-tests`, which is included
in the aggregate `ci-complete` check.
