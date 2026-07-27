# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Smoke test for the assembled ``pm`` surface (dimos/pure/pm.py).

The sketch-floor Tagger defines cleanly via ``pm.*``, classifies STATELESS
with a correct StepSpec, constructs rows, and step runs — zero engine imports.
"""

import subprocess
import sys

from dimos.pure import pm

UNSTAMPED = pm.UNSTAMPED


def _surface() -> set[str]:
    """Public names bound on pm — every one is an `X as X` re-export."""
    return {n for n in vars(pm) if not n.startswith("_")}


def test_pm_surface_exports():
    # The exact surface: T1 rows + T2 §2.3 + T3 §1 + T4 §6, names only. pm has
    # no __all__ (house rule), so the surface IS its public module namespace.
    assert _surface() == {
        # T1 — rows
        "UNSTAMPED",
        "BundleDefinitionError",
        "ContractSpec",
        "FieldSpec",
        "In",
        "InterpolateSpec",
        "LatestSpec",
        "Out",
        "PlainSpec",
        "TickSpec",
        "contract",
        "interpolate",
        "latest",
        "tick",
        # T2 — config
        "ConfigFieldError",
        "FrozenModuleError",
        "PureModule",
        "PureModuleConfig",
        # T3 — stepspec
        "PureModuleDefinitionError",
        "StepKind",
        "StepSpec",
        "step_spec",
        # T4 — typing
        "AsyncStateless",
        "EngineSurface",
        "Fold",
        "InPort",
        "InPorts",
        "Mealy",
        "OutPort",
        "OutPorts",
        "Stamped",
        "Stateless",
        # T5 — align
        "Aligner",
        "AlignmentError",
        "Interpolatable",
        "align",
        "register_interpolator",
        # T6 — drivers
        "PureModuleRunError",
        "RunHooks",
        "StepError",
        "run_over",
        # T7 — resources (pm exports the decorator only; spec §2.3)
        "resource",
        # pm.State — frozen Mealy-state base, completing In/Out/State symmetry
        "State",
        # T11 — tf specifiers (data layer; the engine stays in dimos.pure.tfbuffer)
        "TfOutSpec",
        "TfSpec",
        "tf",
        "tf_out",
        # T15 — debug recorder (flat pm names; reader stays dimos.pure.debugrec)
        "Debug",
        "debug_latest",
        "debug_load",
        # T9 — health (the ONE global row users declare + autoconnect keys on)
        "Health",
    }
    for name in _surface():
        assert not isinstance(getattr(pm, name), type(sys)), f"{name} is a module"
    assert pm.In.__module__ == "dimos.pure.rows"
    assert pm.PureModule.__module__ == "dimos.pure.module"
    assert pm.StepSpec.__module__ == "dimos.pure.stepspec"
    assert pm.Stateless.__module__ == "dimos.pure.typing"
    assert pm.State.__module__ == "dimos.pure.state"


def test_pm_surface_tagger_floor():
    # The sketch's 4-declaration Tagger, spelled entirely through pm.*.
    class Tagger(pm.PureModule):
        class In(pm.In):
            image: float = pm.tick(expect_hz=30)
            pose: float = pm.interpolate()

        class Out(pm.Out):
            located: str = pm.contract(min_hz=10)

        def step(self, i: In) -> Out:
            return Tagger.Out(located=f"bright={i.image:.2f} @ ({i.pose:.2f})")

    # Classified at the class statement by the active T3 seam.
    spec = pm.step_spec(Tagger)
    assert spec.kind is pm.StepKind.STATELESS
    assert spec.in_type is Tagger.In
    assert spec.out_type is Tagger.Out
    assert spec.state_type is None
    assert spec.skips is False
    assert spec.owner is Tagger

    # Rows construct; step runs; tests need no engine (sketch doctrine).
    row = Tagger.In(ts=0.0, image=0.80, pose=1.50)
    out = Tagger().step(row)
    assert out.located == "bright=0.80 @ (1.50)"
    assert out.ts == UNSTAMPED  # engine stamps later; construction leaves it


def test_pm_surface_definition_errors_are_loud():
    # The unified definition-error type catches both shape and config failures.
    try:

        class Bad(pm.PureModule):
            pass  # no step

    except pm.PureModuleDefinitionError as e:
        assert "[step-missing]" in str(e)
    else:  # pragma: no cover
        raise AssertionError("step-less PureModule subclass must not define")


def test_pm_surface_zero_engine_imports():
    # Importing the surface pulls no engine/runtime machinery — pinned in a
    # fresh interpreter so sibling tests cannot pollute sys.modules.
    code = (
        "import sys\n"
        "import dimos.pure\n"
        "banned = [m for m in sys.modules if m.startswith(("
        "'dimos.core', 'dimos.stream', 'dimos.robot', 'numpy', 'cv2', 'torch', 'lcm', 'zenoh'"
        "))]\n"
        "assert not banned, banned\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
