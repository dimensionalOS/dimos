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

"""Scheduling loop for characterization runs.

One run produces three artifacts in an output directory:

    {run_dir}/
        run.json              — run metadata (written before start, finalized on stop)
        cmd_monotonic.jsonl   — one line per commanded sample, monotonic-clock timed
        recording.db          — memory2 SQLite DB (written by CharacterizationRecorder)

The runner owns timing (monotonic-clock busy-wait) and command generation.
The Recorder (running inside a coordinator blueprint) snoops the same
``/cmd_vel`` and ``/go2/odom`` LCM topics and records them to SQLite on
wall-clock ``ts``. We align the two clocks at run start by recording
``(time.monotonic(), time.time())`` in ``run.json``.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.utils.characterization.recipes import TestRecipe
    from dimos.utils.characterization.recorder import BmsLogger

logger = logging.getLogger(__name__)


def _git_sha(repo_root: Path | None = None) -> str | None:
    """Return the current commit SHA, or ``None`` if git isn't available."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
            cwd=repo_root,
            timeout=2.0,
        )
        return out.stdout.strip() or None
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return None


def _generate_run_id(recipe_name: str) -> str:
    """Timestamp-prefixed, recipe-name-suffixed id. Matches dimos run_registry style."""
    stamp = time.strftime("%Y%m%d-%H%M%S")
    safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in recipe_name)
    return f"{stamp}-{safe}"


@dataclass
class RunResult:
    run_id: str
    run_dir: Path
    n_commanded: int
    exit_reason: str
    run_json: Path
    recording_db: Path
    cmd_monotonic_jsonl: Path


@dataclass
class OperatorMetadata:
    """Operator-supplied run context. All fields optional; all stored in run.json."""

    surface: str | None = None
    payload_kg: float | None = None
    gait_mode: str | None = None
    notes: str | None = None
    ground_truth_source: str = "go2_onboard_odom"
    extra: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "surface": self.surface,
            "payload_kg": self.payload_kg,
            "gait_mode": self.gait_mode,
            "notes": self.notes,
            "ground_truth_source": self.ground_truth_source,
        }
        out.update(dict(self.extra))
        return out


class CharacterizationSession:
    """Run a ``TestRecipe`` once. Publishes Twists, writes artifacts, returns a ``RunResult``.

    Construct once, call :meth:`run` once per recipe. The LCM transport
    is started lazily on the first publish, so constructing a session is
    cheap and side-effect-free.
    """

    def __init__(
        self,
        *,
        cmd_vel_topic: str = "/cmd_vel",
        output_root: Path | str,
        bms: BmsLogger | None = None,
    ) -> None:
        self._cmd_vel = LCMTransport(cmd_vel_topic, Twist)
        self._cmd_vel_topic = cmd_vel_topic
        self._output_root = Path(output_root).expanduser().resolve()
        self._output_root.mkdir(parents=True, exist_ok=True)
        self._bms = bms
        self._closed = False

    def close(self) -> None:
        """Stop the LCM transport. Safe to call multiple times."""
        if self._closed:
            return
        try:
            self._cmd_vel.stop()
        except Exception:  # pragma: no cover
            logger.exception("CharacterizationSession: transport stop failed")
        self._closed = True

    def __enter__(self) -> CharacterizationSession:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    def run(
        self,
        recipe: TestRecipe,
        *,
        blueprint_name: str = "unitree_go2_characterization",
        simulation: bool = False,
        operator: OperatorMetadata | None = None,
        run_dir: Path | None = None,
        session_db_path: Path | None = None,
        session_id: str | None = None,
    ) -> RunResult:
        """If ``run_dir`` is given, use it (must already exist); else create one.

        ``session_db_path`` points at a session-level memory2 DB shared by
        all runs in a session. When provided, the run's ``run.json`` stores
        the relative path to that DB and a ``ts_window_wall`` so analysis
        can slice the shared DB to this run's data.
        """
        operator = operator or OperatorMetadata()
        if run_dir is not None:
            run_dir = Path(run_dir).expanduser().resolve()
            run_id = run_dir.name
        else:
            run_id = _generate_run_id(recipe.name)
            run_dir = self._output_root / run_id
            run_dir.mkdir(parents=True, exist_ok=False)

        run_json = run_dir / "run.json"
        cmd_jsonl = run_dir / "cmd_monotonic.jsonl"
        recording_db = run_dir / "recording.db"

        # Clock alignment anchor written before the first command.
        t_mono_start = time.monotonic()
        t_wall_start = time.time()

        bms_start = self._bms.snapshot() if self._bms is not None else None

        # Session-DB plumbing: store the relative path so the run dir is
        # relocatable as long as the session root is kept together.
        session_db_rel: str | None = None
        if session_db_path is not None:
            try:
                session_db_rel = str(
                    Path(session_db_path).resolve().relative_to(run_dir.resolve(), walk_up=True)
                )
            except (ValueError, TypeError):
                session_db_rel = str(Path(session_db_path).resolve())

        metadata_head = {
            "run_id": run_id,
            "session_id": session_id,
            "recipe": recipe.serialize(),
            "blueprint": blueprint_name,
            "simulation": simulation,
            "cmd_vel_topic": self._cmd_vel_topic,
            "started_at_wall": t_wall_start,
            "started_at_monotonic": t_mono_start,
            "clock_anchor": {"monotonic": t_mono_start, "wall": t_wall_start},
            "operator": operator.as_dict(),
            "git_sha": _git_sha(),
            "python_version": sys.version.split()[0],
            "dimos_version": _dimos_version(),
            "bms_start": bms_start,
            # One of these will be the measured-data source at analysis time.
            "recording_db": recording_db.name if session_db_rel is None else None,
            "session_db_path": session_db_rel,
            "cmd_monotonic_jsonl": cmd_jsonl.name,
            "bms_samples": [],  # appended below; filled in at finalize time
        }
        _write_json(run_json, metadata_head)

        exit_reason = "ok"
        n_commanded = 0
        bms_samples: list[dict[str, Any]] = []
        last_bms_mono: float = -1.0
        try:
            with cmd_jsonl.open("w") as fh:
                total = recipe.pre_roll_s + recipe.duration_s + recipe.post_roll_s
                dt = 1.0 / recipe.sample_rate_hz
                seq = 0
                t_start = time.monotonic()

                while True:
                    t_mono = time.monotonic()
                    t_rel = t_mono - t_start
                    if t_rel >= total:
                        break

                    # Phase: pre-roll [0, pre_roll_s); active [pre, pre+dur); post-roll after.
                    if t_rel < recipe.pre_roll_s:
                        vx, vy, wz = 0.0, 0.0, 0.0
                        phase = "pre_roll"
                    elif t_rel < recipe.pre_roll_s + recipe.duration_s:
                        t_active = t_rel - recipe.pre_roll_s
                        vx, vy, wz = recipe.signal_fn(t_active)
                        phase = "active"
                    else:
                        vx, vy, wz = 0.0, 0.0, 0.0
                        phase = "post_roll"

                    twist = Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))
                    self._cmd_vel.publish(twist)
                    t_wall = time.time()

                    fh.write(
                        json.dumps(
                            {
                                "seq": seq,
                                "tx_mono": t_mono,
                                "tx_wall": t_wall,
                                "phase": phase,
                                "vx": vx,
                                "vy": vy,
                                "wz": wz,
                            }
                        )
                        + "\n"
                    )
                    n_commanded += 1
                    seq += 1

                    # BMS at ~1 Hz
                    if self._bms is not None and self._bms.available:
                        if t_mono - last_bms_mono >= 1.0:
                            snap = self._bms.snapshot()
                            snap["t_mono"] = t_mono
                            snap["t_wall"] = t_wall
                            bms_samples.append(snap)
                            last_bms_mono = t_mono

                    next_t = t_start + (seq * dt)
                    sleep_s = next_t - time.monotonic()
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                # One last zero-twist kick to guarantee the plant sees 0 on shutdown.
                self._cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

        except KeyboardInterrupt:
            exit_reason = "interrupted"
            logger.warning("run %s interrupted by user", run_id)
        except Exception as e:
            exit_reason = f"exception:{type(e).__name__}:{e}"
            logger.exception("run %s failed", run_id)
        finally:
            bms_end = self._bms.snapshot() if self._bms is not None else None
            t_wall_end = time.time()
            metadata_head["completed_at_wall"] = t_wall_end
            metadata_head["completed_at_monotonic"] = time.monotonic()
            metadata_head["exit_reason"] = exit_reason
            metadata_head["n_commanded"] = n_commanded
            metadata_head["bms_end"] = bms_end
            metadata_head["bms_samples"] = bms_samples
            # Wall-clock window for session-DB slicing at analysis time.
            # Pad by 200 ms on each side so we don't clip samples arriving
            # right at the edge due to transport/callback delay.
            metadata_head["ts_window_wall"] = {
                "start": t_wall_start - 0.2,
                "end": t_wall_end + 0.2,
            }
            _write_json(run_json, metadata_head)

        return RunResult(
            run_id=run_id,
            run_dir=run_dir,
            n_commanded=n_commanded,
            exit_reason=exit_reason,
            run_json=run_json,
            recording_db=recording_db,
            cmd_monotonic_jsonl=cmd_jsonl,
        )


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as fh:
        json.dump(data, fh, indent=2, default=str)
        fh.write("\n")
    os.replace(tmp, path)


def _dimos_version() -> str | None:
    try:
        from importlib.metadata import version

        return version("dimos")
    except Exception:
        return None


__all__ = ["CharacterizationSession", "OperatorMetadata", "RunResult"]
