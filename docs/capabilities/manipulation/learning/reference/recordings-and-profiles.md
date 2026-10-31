# Recordings and profiles

## Recording directory

```text
recordings/session-001/
├── schema.json
└── recording.mcap
```

Choose `--recorder.format sqlite` for `recording.db` instead. A new directory
is required; existing directories are never overwritten or resumed. Copy or move
the whole directory.

The collection layer writes the schema before capture. It contains the relative
payload filename, profile identity, dataset features, joint ordering, episode
extraction, synchronization, and quality settings. No Python classes or absolute
dataset paths are serialized. An interrupted session remains available for
inspection; incomplete and discarded episodes are not exported.

## Profile declarations

`CollectionProfile` has required `observations` and `actions` mappings and optional
`metadata`. Feature declarations include the source stream, Python message type,
field, dtype, shape, joint names, and source kind. Shared-source projections are
recorded once and must agree on message type and source kind.

Profiles have no separate registry. Blueprint discovery uses the built-in
registry and installed `dimos.blueprints` entry points. Metadata names identify
recordings; they do not select Blueprints or hardware.

The collection API fixes synchronization at 30 Hz with 20 ms tolerance. Its
anchor is the first video observation, or the first observation if there is no
video. It exposes no sync or quality configuration. The episode monitor controls
start, save, and discard; conversion validates recorded values.

## Source semantics and alignment

Each feature declares its recorded source's meaning with `source_kind`:

- `"snapshot"` (default): align to the nearest observation within the configured
  tolerance. This also applies when measured state supplies a teaching action.
- `"joint_position_updates"`: reconstruct persistent `JointState.position`
  targets by joint name, using only updates at or before each dataset timestamp.
  Omitted joints retain their targets, including across episode boundaries.
  Missing initial joints and malformed updates fail validation.

Features sharing a recorded stream must declare the same source kind. Command
history is reconstructed once before projecting individual features. Inspection
and preparation share alignment and value checks; MCAP and SQLite capture remain
unaligned, native-rate streams. Start a new recording after an unrecorded target
reset or control-mode change.

Quest collection actions come from the main coordinator's
`coordinator_joint_command` stream. They are attempted commands, not evidence of
controller acceptance or physical arrival. Hand teaching uses measured joint
positions as both observation and action.

## LeRobot feature names

Conversion maps a single numeric observation to `observation.state`, image
observations to `observation.images.<name>`, and a single action to `action`.
Multiple numeric observations use `observation.<name>`; multiple actions use
`action.<name>`. Names already following these conventions stay unchanged.
Collisions are errors. The mapping applies to feature metadata and every frame.

`dimos_meta.json` preserves declarations and the computed mapping. It cannot
replace standard dataset keys in `meta/info.json`. Conversion does not
concatenate vectors or establish compatibility with every policy. HDF5 retains
configured feature names.

## Python preparation API

`RecordingSchema` is the public saved schema, imported from
`dimos.imitation.dataprep.schema`. `DataPrepConfig` composes a recording schema
with source and output settings. `RecordingSchema.read(directory)` reads a saved
schema; `schema.dataprep_config(directory, OutputConfig(...))` creates the
conversion configuration.

`run_lerobot_dataprep(config)` performs conversion in the isolated LeRobot
project. `run_dataprep(config)` supports HDF5 output. Native MCAP and SQLite
recording preserve asynchronous streams; LeRobot conversion is a separate step.

See [offline preparation and export](/dimos/imitation/dataprep/README.md)
for API details and [collection examples](/dimos/imitation/collection/README.md)
for typed source wiring.
