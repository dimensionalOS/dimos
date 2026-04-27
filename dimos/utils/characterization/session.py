# Copyright 2025-2026 Dimensional Inc.
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

"""Session manager — one coordinator, many recipes, one shared DB.

This is the "design B" path: start the blueprint once, keep it alive
across every recipe in the session, and let all runs write into a
single session-level memory2 SQLite. Per-run dirs hold only the
commanded-sample JSONL and ``run.json`` with a pointer to the session
DB plus a wall-clock window so analysis can slice that DB to just this
run's observations.

Typical flow (used by ``python -m dimos.utils.characterization.scripts.run_session``):

    with SessionManager.build(plan, output_root=...) as mgr:
        mgr.start_coordinator()
        for entry in mgr.plan:
            mgr.prompt_operator(entry)   # ENTER / s / r / q — teleop in between
            mgr.run(entry)
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.utils.characterization.recorder import BmsLogger, CharacterizationRecorder
from dimos.utils.characterization.recipes import TestRecipe
from dimos.utils.characterization.runner import (
    CharacterizationSession,
    OperatorMetadata,
    RunResult,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = logging.getLogger(__name__)


def _session_id() -> str:
    return f"session_{time.strftime('%Y%m%d-%H%M%S')}"


@dataclass(frozen=True)
class PlannedRun:
    """One (recipe, repeat_index) entry in a session plan."""

    recipe: TestRecipe
    repeat_index: int  # 1-based within the repeats of this recipe
    repeat_total: int  # total repeats requested for this recipe

    @property
    def label(self) -> str:
        """Filesystem-safe label used for the run dir name."""
        safe = "".join(
            c if (c.isalnum() or c in "-_.") else "_" for c in self.recipe.name
        )
        return f"{safe}_r{self.repeat_index}of{self.repeat_total}"


def expand_plan(
    entries: Iterable[tuple[TestRecipe, int]],
    *,
    randomize: bool = False,
    rng_seed: int | None = None,
) -> list[PlannedRun]:
    """Turn ``[(recipe, repeats), ...]`` into a flat list of ``PlannedRun``.

    ``randomize`` shuffles the expanded list; pass ``rng_seed`` for
    reproducible sessions (e.g. for A/B tests). Randomization runs after
    expansion so each repeat is an independent slot in the permutation.
    """
    expanded: list[PlannedRun] = []
    for recipe, repeats in entries:
        if repeats <= 0:
            continue
        for i in range(1, repeats + 1):
            expanded.append(PlannedRun(recipe=recipe, repeat_index=i, repeat_total=repeats))
    if randomize:
        import random

        r = random.Random(rng_seed)
        r.shuffle(expanded)
    return expanded


def build_session_blueprint(
    db_path: Path,
    *,
    backend: str = "go2",
    include_teleop: bool = True,
    rage: bool = False,
) -> Blueprint:
    """Compose the session blueprint: coordinator + recorder (+ optional teleop).

    Returns a Blueprint with the Recorder pointed at ``db_path``; the
    caller builds a ``ModuleCoordinator`` from it.

    When ``include_teleop`` is True, we add the standard
    ``KeyboardTeleop`` module (runs in its own worker process, so
    pygame rendering doesn't contend with the control tick loop). It's
    configured with ``publish_only_when_active=True`` so its output
    stream is silent when no motion key is held — otherwise its 50Hz
    zero-Twist stream would fight with the recipe runner's commands on
    ``/cmd_vel``.

    When ``rage`` is True (go2 backend only), patches the GO2Connection
    blueprint atom to ``mode="rage"`` so the connection's start path
    runs StandUp → BalanceStand → enable_rage_mode after connect.
    Characterizes a different plant: faster locomotion envelope.
    """
    if backend == "go2":
        from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
            unitree_go2_coordinator as base,
        )
    elif backend == "mock":
        if rage:
            raise ValueError("--rage is only valid with --backend go2")
        from dimos.control.blueprints.mobile import coordinator_mock_twist_base as base
    else:
        raise ValueError(f"unknown backend: {backend!r}")

    if rage:
        base = _patch_go2_mode(base, mode="rage")

    transports: dict[tuple[str, type], Any] = {
        ("commanded", Twist): LCMTransport("/cmd_vel", Twist),
    }
    if backend == "go2":
        transports[("measured", PoseStamped)] = LCMTransport("/go2/odom", PoseStamped)

    atoms = [CharacterizationRecorder.blueprint(db_path=str(db_path))]

    if include_teleop:
        from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

        # Match rage_teleop's higher speeds when characterizing the rage
        # envelope so the operator's manual repositioning between runs
        # isn't laggy compared to the recipe-driven motion.
        teleop_kwargs: dict[str, Any] = {"publish_only_when_active": True}
        if rage:
            teleop_kwargs["linear_speed"] = 1.25
            teleop_kwargs["angular_speed"] = 1.2

        atoms.append(KeyboardTeleop.blueprint(**teleop_kwargs))

    bp = autoconnect(base, *atoms).transports(transports)
    return bp


def _patch_go2_mode(bp: Blueprint, *, mode: str) -> Blueprint:
    """Return a copy of ``bp`` with the GO2Connection atom's kwargs updated
    to include ``mode=<mode>`` (e.g. "rage").

    The stock ``unitree_go2_coordinator`` calls ``GO2Connection.blueprint()``
    with no kwargs (mode defaults to freewalk). We need rage without
    duplicating the whole blueprint, so we mutate the atom's kwargs.
    """
    from dataclasses import replace

    from dimos.robot.unitree.go2.connection import GO2Connection

    new_atoms = []
    touched = False
    for atom in bp.blueprints:
        if atom.module is GO2Connection:
            new_kwargs = dict(atom.kwargs)
            new_kwargs["mode"] = mode
            new_atoms.append(replace(atom, kwargs=new_kwargs))
            touched = True
        else:
            new_atoms.append(atom)
    if not touched:
        logger.warning(
            "Mode patch: no GO2Connection atom found in blueprint, skipping (mode=%s)", mode
        )
        return bp
    return replace(bp, blueprints=tuple(new_atoms))


@dataclass
class SessionResult:
    session_id: str
    session_dir: Path
    session_db: Path
    session_json: Path
    runs: list[RunResult] = field(default_factory=list)
    aborted: bool = False


class SessionManager:
    """Own the coordinator, recorder, and session-level artifacts across many recipes."""

    def __init__(
        self,
        *,
        session_id: str,
        session_dir: Path,
        plan: list[PlannedRun],
        backend: str,
        simulation: bool,
        include_teleop: bool,
        warmup_s: float,
        operator: OperatorMetadata,
        rage: bool = False,
    ) -> None:
        self.session_id = session_id
        self.session_dir = session_dir
        self.session_db = session_dir / "recording.db"
        self.session_json = session_dir / "session.json"
        self.plan = plan
        self.backend = backend
        self.simulation = simulation
        self.include_teleop = include_teleop
        self.warmup_s = warmup_s
        self.operator = operator
        self.rage = rage

        self._coord: ModuleCoordinator | None = None
        self._recipe_session: CharacterizationSession | None = None
        self._bms: BmsLogger | None = None
        self._closed = False
        self._runs: list[RunResult] = []
        self._aborted = False

    @classmethod
    def build(
        cls,
        plan: list[PlannedRun],
        *,
        output_root: Path | str,
        backend: str = "go2",
        simulation: bool = False,
        include_teleop: bool = True,
        warmup_s: float = 4.0,
        operator: OperatorMetadata | None = None,
        session_id: str | None = None,
        rage: bool = False,
    ) -> SessionManager:
        sid = session_id or _session_id()
        out_root = Path(output_root).expanduser().resolve()
        out_root.mkdir(parents=True, exist_ok=True)
        sdir = out_root / sid
        sdir.mkdir(parents=True, exist_ok=False)
        return cls(
            session_id=sid,
            session_dir=sdir,
            plan=plan,
            backend=backend,
            simulation=simulation,
            include_teleop=include_teleop,
            warmup_s=warmup_s,
            operator=operator or OperatorMetadata(),
            rage=rage,
        )

    def __enter__(self) -> SessionManager:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    # -------------------------------------------------------------------- lifecycle

    def start_coordinator(self) -> None:
        """Spin up the blueprint. Blocks for ``warmup_s`` before returning."""
        if self._coord is not None:
            return
        if self.simulation:
            global_config.update(simulation=True)

        bp = build_session_blueprint(
            self.session_db,
            backend=self.backend,
            include_teleop=self.include_teleop,
            rage=self.rage,
        )
        self._write_session_head(status="booting")
        logger.info("session %s: building blueprint (%s%s)",
                    self.session_id, self.backend, " [sim]" if self.simulation else "")
        self._coord = ModuleCoordinator.build(bp)
        time.sleep(self.warmup_s)

        self._recipe_session = CharacterizationSession(
            cmd_vel_topic="/cmd_vel",
            output_root=self.session_dir,
            bms=self._try_make_bms_logger(),
        )
        self._write_session_head(status="ready")

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._recipe_session is not None:
                self._recipe_session.close()
        except Exception:  # pragma: no cover
            logger.exception("session %s: recipe session close failed", self.session_id)

        try:
            if self._coord is not None:
                logger.info("session %s: stopping coordinator...", self.session_id)
                self._coord.stop()
        except Exception:  # pragma: no cover
            logger.exception("session %s: coordinator stop failed", self.session_id)

        try:
            self._write_session_head(status="closed")
        except Exception:  # pragma: no cover
            logger.exception("session %s: final session.json write failed", self.session_id)

        self._closed = True

    # -------------------------------------------------------------------- recipe

    def run(self, entry: PlannedRun, *, run_index: int) -> RunResult:
        if self._recipe_session is None:
            raise RuntimeError("SessionManager.run() called before start_coordinator()")

        run_dir = self.session_dir / f"{run_index:03d}_{entry.label}"
        run_dir.mkdir(parents=True, exist_ok=False)
        result = self._recipe_session.run(
            entry.recipe,
            blueprint_name=f"{self.backend}_characterization",
            simulation=self.simulation,
            operator=self.operator,
            run_dir=run_dir,
            session_db_path=self.session_db,
            session_id=self.session_id,
        )
        self._runs.append(result)
        self._write_session_head(status="running")
        return result

    # -------------------------------------------------------------------- helpers

    def _try_make_bms_logger(self) -> BmsLogger | None:
        if self._coord is None:
            return None
        try:
            from dimos.robot.unitree.go2.connection import GO2Connection
        except Exception:
            return None
        try:
            go2 = self._coord.get_module(GO2Connection)
        except Exception:
            return None
        inner = getattr(go2, "connection", None)
        if inner is None or not hasattr(inner, "lowstate_stream"):
            return None
        return BmsLogger(inner)

    def _write_session_head(self, *, status: str) -> None:
        data: dict[str, Any] = {
            "session_id": self.session_id,
            "session_dir": str(self.session_dir),
            "backend": self.backend,
            "simulation": self.simulation,
            "rage": self.rage,
            "warmup_s": self.warmup_s,
            "operator": self.operator.as_dict(),
            "status": status,
            "plan": [
                {
                    "label": p.label,
                    "recipe": p.recipe.serialize(),
                    "repeat_index": p.repeat_index,
                    "repeat_total": p.repeat_total,
                }
                for p in self.plan
            ],
            "runs": [
                {
                    "run_id": r.run_id,
                    "run_dir": str(r.run_dir),
                    "exit_reason": r.exit_reason,
                    "n_commanded": r.n_commanded,
                }
                for r in self._runs
            ],
            "aborted": self._aborted,
            "updated_at_wall": time.time(),
        }
        tmp = self.session_json.with_suffix(".json.tmp")
        with tmp.open("w") as fh:
            json.dump(data, fh, indent=2, default=str)
            fh.write("\n")
        os.replace(tmp, self.session_json)

    def to_result(self) -> SessionResult:
        return SessionResult(
            session_id=self.session_id,
            session_dir=self.session_dir,
            session_db=self.session_db,
            session_json=self.session_json,
            runs=list(self._runs),
            aborted=self._aborted,
        )

    def mark_aborted(self) -> None:
        self._aborted = True
        self._write_session_head(status="aborted")


__all__ = [
    "PlannedRun",
    "SessionManager",
    "SessionResult",
    "build_session_blueprint",
    "expand_plan",
]
