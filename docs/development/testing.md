# Testing

`uv run` syncs the project deps + `tests` group on demand, so the default test suite needs no upfront install — just `uv run pytest --numprocesses=auto dimos` (xdist parallelizes across cores).

Self-hosted tests need the heavy optional extras (LFS data, perception models, simulation, hardware SDKs, …). Sync them explicitly before running:

```bash
uv sync --all-extras              # everything the resolver supports for your platform
uv sync --group tests-self-hosted # just what CI installs on the self-hosted runner
```

## Types of tests

In general, there are different types of tests based on what their goal is:

| Type | Description | Mocking | Speed |
|------|-------------|---------|-------|
| Unit | Test a small individual piece of code | All external systems | Very fast |
| Integration | Test the integration between multiple units of code | Most external systems | Some fast, some slow |
| Functional | Test a particular desired functionality | Some external systems | Some fast, some slow |
| End-to-end | Test the entire system as a whole from the perspective of the user | None | Very slow |

The distinction between unit, integration, and functional tests is often debated and rarely productive.

Rather than waste time on classifying tests, it's better to separate tests by how they are used:

| Test Group | When to run | Typical usage |
|------------|-------------|---------------|
| **default** | after each code change | often run with filesystem watchers so tests rerun whenever a file is saved |
| **self-hosted** | every once in a while to make sure you haven't broken anything | maybe every commit, but definitely before publishing a PR |

The purpose of running tests in a loop is to get immediate feedback. The faster the loop, the easier it is to identify a problem since the source is the tiny bit of code you changed.

Self-hosted tests are marked with `@pytest.mark.self_hosted` (they need LFS, ROS, CUDA, or other heavy deps); the default suite is everything else.

## Usage

### Default suite

```bash
./bin/pytest-fast
```

This is the same as:

```bash
pytest --numprocesses=auto dimos
```

The default `addopts` in `pyproject.toml` includes a `-m` filter that excludes `self_hosted`/`mujoco`/`tool`, so plain `pytest dimos` runs only the default suite; `--numprocesses=auto` parallelizes across cores via pytest-xdist.

### Self-hosted tests

```bash
./bin/pytest-slow
```

(Shortcut for `pytest --numprocesses=auto -m 'not (tool or mujoco)' dimos` — runs the default suite *and* self-hosted tests, but not `tool` or `mujoco`.)

When writing or debugging a specific self-hosted test, override `-m` yourself to run it:

```bash
pytest -m self_hosted dimos/path/to/test_something.py
```

## Writing tests

Test files live next to the code they test. If you have `dimos/core/pubsub.py`, its tests go in `dimos/core/test_pubsub.py`.

When writing tests you probably want to limit the run to whatever tests you're writing:

```bash
pytest -sv dimos/core/test_my_code.py
```

### Fixtures

Pytest fixtures are very useful for making sure test failures don't affect other tests.

Whenever you have something that needs to be cleaned up when the test is over (disconnect, close, delete temp files, etc.), you should use a fixture.

Simple example code:

```python
import pytest


class RobotArm:
    def __init__(self, device: str) -> None:
        self.device = device
        self._position = (0.0, 0.0, 0.0)

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    def move_to(self, x: float, y: float, z: float) -> None:
        self._position = (x, y, z)

    @property
    def position(self) -> tuple[float, float, float]:
        return self._position


@pytest.fixture
def arm():
    arm = RobotArm(device="/dev/ttyUSB0")
    arm.connect()
    yield arm
    arm.disconnect()

def test_arm_moves_to_position(arm):
    arm.move_to(x=0.5, y=0.3, z=0.1)
    assert arm.position == (0.5, 0.3, 0.1)
```

The `yield` is key: everything before it is setup, everything after is teardown. The teardown runs even if the test fails, so you never leak resources between tests.

### Mocking

It's easier to use the `mocker` fixture instead of `unittest.mock`. It automatically undoes all patches when the test ends, so you don't need `with` blocks.

Patching a method:

```python
def test_uses_cached_position(mocker):
    mocker.patch("dimos.hardware.RobotArm.get_position", return_value=(0.0, 0.0, 0.0))
    arm = RobotArm()
    assert arm.get_position() == (0.0, 0.0, 0.0)
```

There are other useful things in `mocker`, like `mocker.MagicMock()` for creating fake objects.

## Useful pytest options

| Option | Description |
|--------|-------------|
| `-s` | Show stdout/stderr output |
| `-v` | More verbose test names |
| `-x` | Stop on first failure |
| `-k foo` | Only run tests matching `foo` |
| `--lf` | Rerun only the tests that failed last time |
| `--pdb` | Drop into the debugger when a test fails |
| `--tb=short` | Shorter tracebacks |
| `--durations=0` | Measure the speed of each test |

## Markers

We have a few markers in use now.

* `self_hosted`: used to mark tests that need the self-hosted runner (LFS, ROS, CUDA, heavy deps).
* `tool`: tests which require human interaction. I don't like this. Please don't use them.
* `mujoco`: tests which use `MuJoCo`. These are very slow and don't work in CI currently.

If a test needs to be skipped for some reason, please use on of these markers, or add another one.

* `skipif_in_ci`: tests which cannot run in GitHub Actions
* `skipif_no_openai`: tests which require an `OPENAI_API_KEY` key in the env
* `skipif_no_alibaba`: tests which require an `ALIBABA_API_KEY` key in the env
