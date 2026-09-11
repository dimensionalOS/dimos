# Imitation learning

See [Imitation Learning for Manipulation](../../docs/capabilities/manipulation/imitation-learning.md)
for collection, custom profiles, dataset preparation, and existing LeRobot rollout.

- `collection/profile.py`: typed source streams and dataset feature projections.
- `collection/native_recorder.py`: profile-to-Blueprint recorder factory.
- `collection/recording.py`: portable recording directory and saved schema.
- `dataprep/`: MCAP and SQLite preparation for LeRobot or HDF5.
- `tui.py`: attached episode and rollout controls, discovered through typed Specs.
- `policy/lerobot/`: isolated single-camera policy runtime.

The recorder declares ports before autoconnect. Robot Blueprints construct the
hardware, cameras, and transports; profiles can declare any number of cameras.
Use ordinary `dimos run` configuration and external Blueprint entry points.
The imitation CLI does not maintain a separate workflow registry.
