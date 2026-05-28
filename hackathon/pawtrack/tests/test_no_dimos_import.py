"""Guard: the pure modules must not import DimOS.

The pure logic (state machines, geometry, metrics) is the part that stays
testable without hardware or heavy models. Importing any of it must not drag in
the ``dimos`` package. This runs in a fresh subprocess because other tests in the
suite legitimately import DimOS, so ``dimos`` is already in this process's
``sys.modules`` by the time this test runs.
"""

import os
import subprocess
import sys

_PURE_MODULES = (
    "pawtrack.track_state",
    "pawtrack.motion_fallback",
    "pawtrack.ground_raycast",
    "pawtrack.greeter_state",
    "pawtrack.visited_registry",
    "pawtrack.approach_geometry",
    "pawtrack.identify",
    "pawtrack.occupancy",
)


def test_pure_modules_do_not_import_dimos():
    code = (
        "import importlib, sys\n"
        f"for m in {_PURE_MODULES!r}:\n"
        "    importlib.import_module(m)\n"
        "leaked = sorted(k for k in sys.modules if k.split('.')[0] == 'dimos')\n"
        "assert not leaked, leaked\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.abspath("src")
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr
