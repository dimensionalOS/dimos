# Handoff: RoboPlan `libgomp` static TLS failure on G1

## Status

Confirmed on 2026-08-26 on the G1's Ubuntu aarch64 computer with Python 3.12
and RoboPlan 0.6.0. The failure occurs in a dimOS forkserver worker. RoboPlan
imports successfully in the parent process.

This is a native-wheel and process-load-order problem. It is not a missing
Python dependency, a DDS failure, or a robot-control failure.

## Operator impact

`unitree-g1-teleop` deploys `G1ManipulationModule`, then stops during module
startup. The coordinator performs its normal safe shutdown, including disabling
motor output. Upper-body planning and teleoperation never become available.

The top-level exception is misleading:

```text
ImportError: RoboPlanWorld requires the optional roboplan dependency.
Install the manipulation extra before selecting the roboplan backend.
```

The chained exception contains the real failure:

```text
roboplan.libs/libgomp-<wheel-hash>.so.1.0.0:
cannot allocate memory in static TLS block
```

## Confirmed environment

- Linux aarch64 on the G1 computer
- CPython 3.12
- RoboPlan 0.6.0 from its manylinux 2.28 aarch64 wheel
- dimOS Python modules run in `multiprocessing` forkserver workers
- Pinocchio uses the platform `libgomp.so.1`
- The RoboPlan wheel carries a renamed private copy of `libgomp`

The exact private filename contains a build hash. Do not encode that hash in a
permanent script or document.

## Reproduction

A clean-process import passes:

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
  uv run --no-sync python -c \
  'import roboplan.core; print("roboplan.core: OK")'
```

Importing the full manipulation dependency graph in the parent also passes:

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
  uv run --no-sync python -c \
  'import dimos.manipulation.manipulation_module; import roboplan.core; print("OK")'
```

The following hardware-free test reproduces the failure inside a real dimOS
worker:

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
uv run --no-sync python -c 'from dimos.core.coordination.python_worker import PythonWorker; from dimos.core.coordination.worker_messages import CallMethodRequest; from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_teleop import G1ManipulationModule; w=PythonWorker(); w.start_process(); a=w.deploy_module(G1ManipulationModule); print(a._send_request_to_worker(CallMethodRequest(a._module_id, "_initialize_planning", (), {}))); w.shutdown()'
```

This test does not connect to DDS or send robot commands.

## Root cause

The process loads two OpenMP runtimes:

```text
Pinocchio ───────────────> /lib/aarch64-linux-gnu/libgomp.so.1
RoboPlan native extension -> roboplan.libs/libgomp-<wheel-hash>.so.1.0.0
```

The RoboPlan wheel repair step renamed and bundled its `libgomp`. The dynamic
loader therefore treats it as a second library rather than reusing the platform
copy already loaded for Pinocchio.

`libgomp` uses static thread-local storage. On aarch64, loading the private copy
late in the forkserver worker exhausts the available static TLS block. A clean
Python process succeeds because it has loaded fewer native libraries before
RoboPlan.

Preloading only `/lib/aarch64-linux-gnu/libgomp.so.1` cannot fix the worker:
RoboPlan's extension names its private, renamed library as a separate dependency.

## Temporary workaround

Preload both copies before starting dimOS. Resolve the private filename from the
active virtual environment instead of copying its wheel hash:

```bash
cd ~/cc/dimos

ROBOPLAN_GOMP="$(find "$PWD/.venv/lib/python3.12/site-packages/roboplan.libs" \
  -maxdepth 1 -name 'libgomp-*.so*' -print -quit)"
test -n "$ROBOPLAN_GOMP"

LD_PRELOAD="$ROBOPLAN_GOMP:/lib/aarch64-linux-gnu/libgomp.so.1" \
  uv run --no-sync dimos \
    --viewer none \
    run unitree-g1-teleop \
    --network-interface eth0
```

This workaround is suitable for diagnosis and short-term robot testing. It is
not the desired operator interface: it exposes a private wheel implementation
detail and may change when RoboPlan is rebuilt.

Run the worker-only reproducer with the same dual preload before trying hardware.
Success reaches `World finalized` and exits without an exception.

## Preferred fix

Publish an aarch64 RoboPlan wheel that depends on the platform
`libgomp.so.1` instead of bundling and renaming a private copy. Pinocchio and
RoboPlan should then share one OpenMP runtime.

Review the wheel-repair configuration and exclude `libgomp.so.1` from vendoring.
After publishing the corrected wheel, update the RoboPlan constraint in dimOS
and remove the dual-preload workaround.

### Wheel acceptance criteria

On Linux aarch64 with Python 3.12:

1. `roboplan.core` imports without `LD_PRELOAD` in a clean process.
2. The worker-only reproducer reaches `World finalized` without `LD_PRELOAD`.
3. RoboPlan's native extensions depend on `libgomp.so.1`; the wheel contains no
   `roboplan.libs/libgomp-*` copy.
4. The G1 teleop stack starts repeatedly without a static TLS error.
5. Planning a representative upper-body motion succeeds.

## dimOS follow-up

Fix the optional-dependency guard independently of the wheel. Both
`roboplan_world.py` and `roboplan_planner.py` catch every `ImportError` raised by
`import roboplan.core` and replace it with an installation hint. This masks
native loader failures and missing transitive libraries.

The guard should distinguish a missing RoboPlan distribution from an import
failure inside an installed distribution. At minimum, include the original
exception text in the top-level error. Add a test that injects a nested
`ImportError` and verifies that its loader detail remains visible.

Do not solve the permanent problem by globally preloading RoboPlan in every
dimOS worker. That would make an optional manipulation backend part of all
worker startup and would leave the duplicate OpenMP runtimes in place.

## Investigation notes

The following hypotheses were eliminated:

- The `manipulation` extra was absent: `importlib.metadata` reported RoboPlan
  0.6.0.
- RoboPlan lacked an aarch64 wheel: the installed native extension imported in
  the parent process.
- The dimOS RoboPlan adapter was invalid: `RoboPlanWorld()` constructed in the
  parent process.
- Importing `ManipulationModule` first caused the failure: that ordering also
  passed in the parent process.

The worker-only reproducer exposed the chained static TLS exception and should
remain the primary regression check for this issue.
