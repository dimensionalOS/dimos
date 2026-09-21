# Imitation learning

See [Imitation Learning for Manipulation](../../docs/capabilities/manipulation/imitation-learning.md)
for collection, custom profiles, dataset preparation, and existing LeRobot rollout.

- `collection/profile.py`: typed source streams and dataset feature projections.
- `collection/recorder.py`: profile-to-Blueprint recorder factory.
- `collection/recording.py`: portable recording directory and saved schema.
- `dataprep/`: MCAP and SQLite preparation for LeRobot or HDF5.
- `tui.py`: attached episode and rollout controls, discovered through typed Specs.
- `policy/lerobot/`: isolated single-camera policy runtime.

The recorder declares ports before autoconnect. Robot Blueprints construct the
hardware, cameras, and transports; profiles can declare any number of cameras.
Use ordinary `dimos run` configuration and external Blueprint entry points.
The imitation CLI does not maintain a separate workflow registry.

`CollectionRecorder` extends `RustRecorder` with collection directory and schema
preparation; both use the same Rust executable. Import `collection_recorder` from
`dimos.imitation.collection.recorder`.

The inherited `store` settings must match the destination derived from `recording`
and `format`; collection requires `on_existing=error` and does not rotate backups.
The xArm and Piper collection blueprints also use this recorder, with timestamped
session directories and SQLite payloads.
