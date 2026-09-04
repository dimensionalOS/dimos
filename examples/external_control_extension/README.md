# External Control Extension Example

This example is shaped like a tiny package outside `dimos/`. It proves that an
external package can register a hardware adapter and a control task, then run
them through the normal `ControlCoordinator` path.

Run the demo and watch the logs:

```bash
uv run python examples/external_control_extension/demo_external_control.py
```

Expected log events include:

- registering `BASE/external_test_base`
- registering task type `external_test_drive`
- constructing and connecting `ExternalTestBaseAdapter`
- creating `ExternalTestDriveTask`
- ticking `ExternalTestDriveTask.compute(...)`
- sending commands through `ExternalTestBaseAdapter.write_velocities(...)`

This example performs no robot I/O. The adapter records commands in memory so
the demo is safe to run locally and in CI.
