#!/usr/bin/env python3
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

"""Convert per-episode `.rrd` files written by ``RerunDataRecorder`` into a
LeRobot v2 dataset directory.

Usage::

    python scripts/datasets/rrd_to_lerobot.py \
        --input data/piper_data_collection/<session>/ \
        --task "pick the red block"

One `.rrd` file becomes exactly one LeRobot episode. Frames are gated on
camera arrival (~30 Hz); joint state and action scalars use Rerun's latest-at
semantics on the same query.

Requires the ``datasets`` extra: ``pip install dimos[manipulation]``.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import logging
from pathlib import Path
import sys
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.policy.contracts.registry import (
    available_contracts,
    get_contract,
)

if TYPE_CHECKING:
    from dimos.manipulation.policy.contract import RobotContract

logger = logging.getLogger("rrd_to_lerobot")

_CAMERA_GATE_ENTITY = "/observation/camera/usb"


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rrd_to_lerobot",
        description="Convert RerunDataRecorder .rrd files into a LeRobot v2 dataset.",
    )
    p.add_argument(
        "--input",
        required=True,
        type=Path,
        help="Path to a single .rrd file or a directory containing episode_*.rrd files.",
    )
    p.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output dataset directory. Default: data/lerobot/<input_dir_name>/",
    )
    p.add_argument(
        "--repo-id",
        type=str,
        default=None,
        help="LeRobot dataset repo id. Default: <contract>/<input_dir_name>",
    )
    p.add_argument(
        "--contract",
        type=str,
        default="piper",
        help=f"Robot contract name. Available: {available_contracts()}",
    )
    p.add_argument("--fps", type=int, default=30, help="Dataset frame rate. Default: 30")

    grp = p.add_mutually_exclusive_group()
    grp.add_argument(
        "--binarize-gripper",
        dest="binarize_gripper",
        action="store_true",
        default=True,
        help="Binarize gripper command using contract threshold (default).",
    )
    grp.add_argument(
        "--no-binarize-gripper",
        dest="binarize_gripper",
        action="store_false",
        help="Preserve raw gripper position; do not binarize or rescale.",
    )
    p.add_argument(
        "--gripper-threshold",
        type=float,
        default=None,
        help="Override the contract's gripper binarization threshold (in [0, 1]).",
    )

    task = p.add_mutually_exclusive_group(required=True)
    task.add_argument(
        "--task", type=str, default=None, help="Task description applied to every episode."
    )
    task.add_argument(
        "--task-per-episode",
        type=Path,
        default=None,
        help='JSONL file with {"episode": "<stem>", "task": "<description>"} per line.',
    )

    p.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    return p


def _resolve_input_files(input_path: Path) -> tuple[str, list[Path]]:
    """Return (session_name, sorted list of rrd files).

    Fails fast with a clear error when input is missing or contains no rrd files.
    """
    if not input_path.exists():
        raise FileNotFoundError(f"--input does not exist: {input_path}")
    if input_path.is_file():
        if input_path.suffix != ".rrd":
            raise ValueError(f"--input file is not a .rrd: {input_path}")
        return input_path.stem, [input_path]
    if not input_path.is_dir():
        raise ValueError(f"--input must be a file or directory: {input_path}")
    rrds = sorted(input_path.glob("episode_*.rrd"))
    if not rrds:
        # Tolerate non-`episode_*` naming as a secondary glob.
        rrds = sorted(input_path.glob("*.rrd"))
    if not rrds:
        raise FileNotFoundError(f"--input directory contains no .rrd files: {input_path}")
    return input_path.name, rrds


def _load_task_per_episode(path: Path, rrd_stems: list[str]) -> dict[str, str]:
    if not path.exists():
        raise FileNotFoundError(f"--task-per-episode file does not exist: {path}")
    tasks: dict[str, str] = {}
    with path.open() as fh:
        for ln, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"--task-per-episode line {ln}: malformed JSON: {exc.msg}"
                ) from exc
            ep = obj.get("episode")
            tk = obj.get("task")
            if not isinstance(ep, str) or not isinstance(tk, str):
                raise ValueError(
                    f"--task-per-episode line {ln}: each line must have string 'episode' and 'task'"
                )
            tasks[ep] = tk
    missing = [s for s in rrd_stems if s not in tasks]
    if missing:
        raise KeyError(
            f"--task-per-episode missing entries for: {missing}. Provided: {sorted(tasks)}"
        )
    return tasks


def _validate_threshold(threshold: float | None) -> None:
    if threshold is None:
        return
    if not (0.0 <= threshold <= 1.0):
        raise ValueError(f"--gripper-threshold must be in [0.0, 1.0], got {threshold}")


def _apply_gripper_overrides(
    contract: RobotContract, *, binarize: bool, threshold: float | None
) -> RobotContract:
    """Return a copy of `contract` with the CLI gripper overrides applied.

    Does not mutate the registered contract instance.
    """
    new_cfg = replace(
        contract.gripper_binarization,
        enabled=binarize,
        threshold=(threshold if threshold is not None else contract.gripper_binarization.threshold),
    )
    # Concrete contracts are dataclasses; use replace if available.
    try:
        return replace(contract, gripper_binarization=new_cfg)  # type: ignore[type-var]
    except TypeError:
        # Fall back to attribute mutation on a shallow copy.
        import copy as _copy

        c = _copy.copy(contract)
        c.gripper_binarization = new_cfg  # type: ignore[misc]
        return c


def _ts_ns(ts_value: Any) -> int:
    """Return a pyarrow log_time cell as integer nanoseconds."""
    # `to_pylist()` yields pandas/Python `Timestamp`s; `.value` is ns since epoch
    # for a pandas Timestamp; for datetime fallback use `int(ts.timestamp() * 1e9)`.
    if hasattr(ts_value, "value"):
        return int(ts_value.value)
    if hasattr(ts_value, "timestamp"):
        return int(ts_value.timestamp() * 1_000_000_000)
    return int(ts_value)


def _collect_scalar_series(chunks: list[Any], entity: str) -> tuple[list[int], list[float]]:
    """Concatenate all chunks for `entity` into sorted (ns_times, values) lists.

    Returns empty lists when no chunk for that entity exists.
    """
    pairs: list[tuple[int, float]] = []
    for ch in chunks:
        if ch.entity_path != entity:
            continue
        rb = ch.to_record_batch()
        if "log_time" not in rb.schema.names:
            continue
        scalars_col = None
        for name in rb.schema.names:
            if name.startswith("Scalars:") or name == "Scalars:scalars":
                scalars_col = name
                break
        if scalars_col is None:
            continue
        times = rb.column("log_time").to_pylist()
        vals = rb.column(scalars_col).to_pylist()
        for t, v in zip(times, vals, strict=False):
            if t is None or v is None:
                continue
            # Each cell is a 1-element list (the rerun "instance" dimension).
            scalar = v[0] if isinstance(v, (list, tuple)) and len(v) >= 1 else v
            pairs.append((_ts_ns(t), float(scalar)))
    pairs.sort(key=lambda p: p[0])
    if not pairs:
        return [], []
    times_out, vals_out = zip(*pairs, strict=False)
    return list(times_out), list(vals_out)


def _collect_camera_series(chunks: list[Any], entity: str) -> list[tuple[int, np.ndarray]]:
    """Concatenate all camera chunks for `entity` into sorted (ns_time, image) tuples."""
    out: list[tuple[int, np.ndarray]] = []
    for ch in chunks:
        if ch.entity_path != entity:
            continue
        rb = ch.to_record_batch()
        if "log_time" not in rb.schema.names:
            continue
        if "Image:buffer" not in rb.schema.names or "Image:format" not in rb.schema.names:
            continue
        times = rb.column("log_time").to_pylist()
        bufs = rb.column("Image:buffer").to_pylist()
        fmts = rb.column("Image:format").to_pylist()
        for t, buf, fmt in zip(times, bufs, fmts, strict=False):
            if t is None or buf is None or fmt is None:
                continue
            # buf is a list of one inner pixel-byte list; fmt is a list of one struct.
            inner = buf[0] if isinstance(buf, (list, tuple)) and buf else buf
            f = fmt[0] if isinstance(fmt, (list, tuple)) and fmt else fmt
            try:
                w = int(f["width"])
                h = int(f["height"])
            except (TypeError, KeyError):
                continue
            arr = np.asarray(inner, dtype=np.uint8)
            if arr.size != h * w * 3:
                # Unexpected channel count or pixel format — let the caller skip.
                continue
            arr = arr.reshape(h, w, 3)
            out.append((_ts_ns(t), arr))
    out.sort(key=lambda p: p[0])
    return out


def _latest_at(times: list[int], values: list[Any], target_ns: int) -> Any | None:
    """Return values[i] where times[i] is the largest entry ≤ target_ns, else None."""
    import bisect

    if not times:
        return None
    # bisect_right − 1 gives the rightmost index ≤ target.
    idx = bisect.bisect_right(times, target_ns) - 1
    if idx < 0:
        return None
    return values[idx]


def _process_rrd(
    rrd_path: Path,
    *,
    contract: RobotContract,
    dataset: Any,
    task: str,
    rrd_module: Any,
) -> int:
    """Process one rrd file. Returns the number of frames added.

    Reads chunks via `rerun.experimental.RrdReader` (rerun-sdk 0.32+),
    builds per-entity log-time-sorted series, then for each camera tick
    does a latest-at lookup on each scalar series to assemble one frame.

    Returns 0 with a logged warning when the file is unreadable, has no
    camera frames, or yields no complete frames.
    """
    reader = rrd_module.RrdReader(str(rrd_path))
    chunks = list(reader.store().stream())

    entities = list(contract.rerun_entities())
    scalar_entities = [e for e in entities if e != _CAMERA_GATE_ENTITY]

    # Pre-collect scalar series once per entity.
    scalar_series: dict[str, tuple[list[int], list[float]]] = {
        e: _collect_scalar_series(chunks, e) for e in scalar_entities
    }
    cam_series = _collect_camera_series(chunks, _CAMERA_GATE_ENTITY)

    if not cam_series:
        return 0

    n_added = 0
    n_skipped_partial = 0
    for cam_ns, image in cam_series:
        row: dict[str, Any] = {_CAMERA_GATE_ENTITY: image}
        partial = False
        for entity in scalar_entities:
            times, values = scalar_series[entity]
            v = _latest_at(times, values, cam_ns)
            if v is None:
                partial = True
                break
            row[entity] = v
        if partial:
            n_skipped_partial += 1
            continue
        frame = contract.from_rerun_row(row)
        # lerobot >= 0.5 expects `task` inside the frame dict; older releases
        # accepted it as an `add_frame(frame, task=...)` kwarg. Try the dict
        # path first (newer API) and fall back to the kwarg.
        frame_with_task = {**frame, "task": task}
        try:
            dataset.add_frame(frame_with_task)
        except TypeError:
            dataset.add_frame(frame, task=task)
        n_added += 1

    if n_skipped_partial:
        logger.info(
            "%s: skipped %d frame(s) with no preceding scalar samples",
            rrd_path.name,
            n_skipped_partial,
        )
    return n_added


def _import_lerobot() -> Any:
    # lerobot 0.3.x moved `lerobot.common.datasets` → `lerobot.datasets`;
    # support both so older pinned envs still work.
    try:
        from lerobot.datasets.lerobot_dataset import (
            LeRobotDataset,  # type: ignore[import-not-found, import-untyped]
        )

        return LeRobotDataset
    except ImportError:
        try:
            from lerobot.common.datasets.lerobot_dataset import (  # type: ignore[import-not-found, import-untyped]
                LeRobotDataset,
            )

            return LeRobotDataset
        except ImportError as exc:
            sys.stderr.write(
                "ERROR: lerobot is not installed.\n"
                "Install the datasets extra: pip install dimos[manipulation]\n"
                f"(import error: {exc})\n"
            )
            sys.exit(2)


def _import_rerun_dataframe() -> Any:
    """Return the rerun module providing chunk-based dataframe access.

    rerun-sdk 0.32 exposes the reader as `rerun.experimental.RrdReader`.
    Earlier 0.30/0.31 builds shipped a different `rerun.dataframe` API that
    we don't currently support.
    """
    try:
        import rerun.experimental as rrx

        if not hasattr(rrx, "RrdReader"):
            raise ImportError("rerun.experimental missing RrdReader (need rerun-sdk >= 0.32)")
        return rrx
    except ImportError as exc:
        sys.stderr.write(
            "ERROR: rerun.experimental.RrdReader is not available.\n"
            "Install rerun-sdk >= 0.32 (the dataset converter requires it): "
            "pip install --upgrade rerun-sdk>=0.32\n"
            f"(import error: {exc})\n"
        )
        sys.exit(2)


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # ── Validation (fail fast, no I/O on outputs) ────────────────────────
    _validate_threshold(args.gripper_threshold)
    session_name, rrd_files = _resolve_input_files(args.input)

    rrd_stems = [p.stem for p in rrd_files]
    tasks_by_stem: dict[str, str] | None = None
    if args.task_per_episode is not None:
        tasks_by_stem = _load_task_per_episode(args.task_per_episode, rrd_stems)

    contract = get_contract(args.contract)
    contract = _apply_gripper_overrides(
        contract,
        binarize=args.binarize_gripper,
        threshold=args.gripper_threshold,
    )

    output_dir = args.output or (Path("data/lerobot") / session_name)
    repo_id = args.repo_id or f"{args.contract}/{session_name}"

    # ── Lazy imports of optional dependencies ────────────────────────────
    LeRobotDataset = _import_lerobot()
    rrd_module = _import_rerun_dataframe()

    # ── Build dataset ────────────────────────────────────────────────────
    # LeRobotDataset.create() wants to mkdir `output_dir` itself with
    # exist_ok=False, so only ensure the parent exists.
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=args.fps,
        features=contract.features(),
        root=output_dir,
    )

    n_episodes_written = 0
    n_episodes_skipped = 0
    for rrd_path in rrd_files:
        resolved_task = args.task if args.task is not None else (tasks_by_stem or {})[rrd_path.stem]
        try:
            n_frames = _process_rrd(
                rrd_path,
                contract=contract,
                dataset=dataset,
                task=resolved_task,
                rrd_module=rrd_module,
            )
        except Exception as exc:
            logger.warning("Skipping %s — failed to read rrd: %s", rrd_path.name, exc)
            n_episodes_skipped += 1
            continue

        if n_frames == 0:
            logger.warning(
                "Skipping %s — no camera frames were emitted (empty or camera-less rrd)",
                rrd_path.name,
            )
            n_episodes_skipped += 1
            continue

        dataset.save_episode()
        n_episodes_written += 1
        logger.info(
            "Wrote episode %d (%d frames) from %s", n_episodes_written, n_frames, rrd_path.name
        )

    if n_episodes_written == 0:
        sys.stderr.write(
            f"ERROR: 0 of {len(rrd_files)} rrd files produced episodes "
            f"(skipped: {n_episodes_skipped}).\n"
        )
        return 1

    consolidate = getattr(dataset, "consolidate", None)
    if callable(consolidate):
        consolidate()

    logger.info(
        "Done — wrote %d episode(s) to %s (skipped %d).",
        n_episodes_written,
        output_dir,
        n_episodes_skipped,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
