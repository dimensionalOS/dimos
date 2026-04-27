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

"""CLI: run a batch of recipes in one coordinator session, pause between each.

Unlike ``python -m dimos.utils.characterization.scripts.run_session`` which boots the full blueprint per
recipe, this command boots it once and holds it up for the whole
session. Between runs it prompts the operator so the robot can be
repositioned. Keyboard teleop (at priority 100, above the recipe
velocity task) is always available during pauses — hold WASD / QE to
drive; release to let the next recipe fire.

Usage::

    python -m dimos.utils.characterization.scripts.run_session \\
        --recipes "dimos.utils.characterization.examples:step_vx_1:3,dimos.utils.characterization.examples:step_wz_1:2" \\
        --simulation --out-dir /tmp/char_runs [--randomize] [--rng-seed 42]

``--recipes`` format: comma-separated ``module:attr[:repeats]``.
``repeats`` defaults to 1.
"""

from __future__ import annotations

import argparse
import importlib
import logging
import sys
import time
from pathlib import Path

from dimos.utils.characterization.recipes import TestRecipe
from dimos.utils.characterization.runner import OperatorMetadata
from dimos.utils.characterization.session import (
    PlannedRun,
    SessionManager,
    expand_plan,
)

logger = logging.getLogger(__name__)


def _resolve_recipe(spec: str) -> TestRecipe:
    if ":" not in spec:
        raise ValueError(
            f"recipe spec {spec!r} must be 'module.path:attribute' "
            "(e.g. dimos.utils.characterization.examples:step_vx_1)"
        )
    module_path, attr = spec.split(":", 1)
    mod = importlib.import_module(module_path)
    recipe = getattr(mod, attr)
    if not isinstance(recipe, TestRecipe):
        raise TypeError(f"{spec} resolved to {type(recipe).__name__}, expected TestRecipe")
    return recipe


def _parse_recipes_arg(arg: str) -> list[tuple[TestRecipe, int]]:
    """Parse ``mod:attr:n,mod:attr:n,...`` into a list of ``(recipe, repeats)``."""
    out: list[tuple[TestRecipe, int]] = []
    for part in arg.split(","):
        part = part.strip()
        if not part:
            continue
        # Split on ':' carefully — module paths don't contain ':'
        tokens = part.split(":")
        if len(tokens) == 2:
            spec, repeats_str = ":".join(tokens), "1"
        elif len(tokens) == 3:
            spec = f"{tokens[0]}:{tokens[1]}"
            repeats_str = tokens[2]
        else:
            raise ValueError(
                f"bad recipe entry {part!r}; expected 'module:attr' or 'module:attr:repeats'"
            )
        try:
            repeats = int(repeats_str)
        except ValueError as e:
            raise ValueError(f"bad repeats in {part!r}: {repeats_str}") from e
        out.append((_resolve_recipe(spec), repeats))
    return out


def _estimate_duration(plan: list[PlannedRun]) -> float:
    """Rough wall-clock estimate for the session: recipe durations + pause slack."""
    total = 0.0
    for p in plan:
        r = p.recipe
        total += r.pre_roll_s + r.duration_s + r.post_roll_s
    return total


def _estimate_forward_distance(plan: list[PlannedRun]) -> float:
    """Very rough per-entry forward-distance estimate (ignores ramps / chirps)."""
    total = 0.0
    for p in plan:
        r = p.recipe
        # Probe the signal at a few points; use vx integral over active window.
        dt = 0.1
        t = 0.0
        vx_integral = 0.0
        while t < r.duration_s:
            vx, _, _ = r.signal_fn(t)
            vx_integral += vx * dt
            t += dt
        total += abs(vx_integral)
    return total


def _prompt(msg: str, valid_keys: str = "") -> str:
    """Stdin prompt. Returns stripped lower-case string or '' on EOF/interrupt."""
    try:
        ans = input(msg).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return "q"
    return ans


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a batch of characterization recipes under one coordinator."
    )
    parser.add_argument(
        "--recipes",
        required=True,
        help=(
            "Comma-separated list of 'module:attr[:repeats]'. "
            "Example: dimos.utils.characterization.examples:step_vx_1:3"
        ),
    )
    parser.add_argument(
        "--out-dir",
        default="/tmp/char_runs",
        help="Parent directory for the session subdir (default /tmp/char_runs)",
    )
    parser.add_argument(
        "--backend",
        choices=("go2", "mock"),
        default="go2",
        help="go2 (real/mujoco) or mock (no-robot plumbing test)",
    )
    parser.add_argument("--simulation", action="store_true", help="Launch mujoco sim for go2")
    parser.add_argument(
        "--rage",
        action="store_true",
        help="Real-Go2 only: enable rage mode at connection start (StandUp -> "
        "BalanceStand -> enable_rage_mode). Characterizes a different plant — "
        "tag your --notes accordingly so the rage dataset stays distinct.",
    )
    parser.add_argument("--randomize", action="store_true", help="Shuffle the expanded plan")
    parser.add_argument("--rng-seed", type=int, default=None, help="Seed for --randomize")
    parser.add_argument("--dry-run", action="store_true", help="Print plan then exit")
    parser.add_argument("--no-teleop", action="store_true", help="Do not add the keyboard teleop task")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Skip the operator prompt between runs (useful in sim)",
    )
    parser.add_argument("--warmup-s", type=float, default=4.0)
    parser.add_argument("--surface", default=None)
    parser.add_argument("--payload-kg", type=float, default=None)
    parser.add_argument("--gait-mode", default=None)
    parser.add_argument("--notes", default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    entries = _parse_recipes_arg(args.recipes)
    if not entries:
        print("no recipes parsed; nothing to do.", file=sys.stderr)
        return 2

    plan = expand_plan(entries, randomize=args.randomize, rng_seed=args.rng_seed)

    # Dry-run: print plan and exit.
    est_dur = _estimate_duration(plan)
    est_fwd = _estimate_forward_distance(plan)
    print(f"session plan ({len(plan)} runs, ~{est_dur:.0f}s command time, ~{est_fwd:.1f}m cumulative forward motion):")
    for i, p in enumerate(plan):
        r = p.recipe
        print(
            f"  [{i:3d}] {p.label}  type={r.test_type}  "
            f"dur={r.duration_s:.1f}s  rate={r.sample_rate_hz:.0f}Hz"
        )
    if args.dry_run:
        return 0

    operator = OperatorMetadata(
        surface=args.surface,
        payload_kg=args.payload_kg,
        gait_mode=args.gait_mode,
        notes=args.notes,
    )

    print()
    print(f"bringing up {args.backend} coordinator{' [sim]' if args.simulation else ''}...")
    with SessionManager.build(
        plan,
        output_root=args.out_dir,
        backend=args.backend,
        simulation=args.simulation,
        include_teleop=(not args.no_teleop) and args.backend == "go2",
        warmup_s=args.warmup_s,
        operator=operator,
        rage=args.rage,
    ) as mgr:
        try:
            mgr.start_coordinator()
        except Exception:
            logger.exception("coordinator failed to start; aborting session")
            mgr.mark_aborted()
            return 3
        print(f"session: {mgr.session_id}  dir: {mgr.session_dir}")
        print("  (hold WASD/QE for keyboard teleop during pauses; release to resume)")

        # Pre-session park: let the operator position the robot before the first
        # run fires. Teleop task is already live; the coordinator is up.
        if not args.auto and mgr.include_teleop:
            print()
            print("Position robot at the start pose, then press ENTER to start the session.")
            ans = _prompt("  [ENTER]=start / q=quit before running anything > ")
            if ans == "q":
                mgr.mark_aborted()
                print("session aborted by operator before first run.")
                return 0

        i = 0
        last_idx: int | None = None
        skipped = 0
        while i < len(plan):
            entry = plan[i]
            r = entry.recipe
            print()
            print(
                f"--- [{i+1}/{len(plan)}] {entry.label}  "
                f"type={r.test_type}  dur={r.duration_s:.1f}s ---"
            )
            if not args.auto:
                ans = _prompt("  [ENTER]=run / s=skip / r=repeat-last / q=quit > ")
                if ans == "q":
                    mgr.mark_aborted()
                    print("session aborted by operator.")
                    break
                if ans == "s":
                    print(f"  skipping {entry.label}")
                    skipped += 1
                    i += 1
                    continue
                if ans == "r":
                    if last_idx is None:
                        print("  no previous run to repeat; running this one.")
                    else:
                        entry = plan[last_idx]
                        print(f"  repeating previous: {entry.label}")
                        # Fall through without advancing i; reuse this slot index
                        # for the repeat's directory so it's clear in the session.

            t_start = time.time()
            try:
                result = mgr.run(entry, run_index=i)
            except Exception:
                logger.exception("run %d (%s) raised unhandled exception", i, entry.label)
                skipped += 1
                i += 1
                continue
            dt = time.time() - t_start
            print(
                f"  -> {result.exit_reason}  samples={result.n_commanded}  "
                f"dir={result.run_dir.name}  elapsed={dt:.1f}s"
            )
            last_idx = i
            i += 1

        print()
        print(f"session done. runs={len(mgr._runs)} skipped={skipped}  session_dir={mgr.session_dir}")
        print("  analyze individual runs:")
        for r in mgr._runs[-3:]:
            print(f"    python -m dimos.utils.characterization.scripts.analyze_run {r.run_dir}")

        # Post-session park: teleop is still live because we haven't stopped
        # the coordinator yet (the `with` block's __exit__ hasn't fired). Give
        # the operator a chance to drive the robot home before everything
        # tears down and the teleop window goes away.
        if not args.auto and mgr.include_teleop:
            print()
            print(
                "Teleop is still active. Drive the robot back to a safe pose, "
                "then press ENTER to shut down the coordinator."
            )
            _prompt("  [ENTER]=shut down > ")

    return 0


if __name__ == "__main__":
    sys.exit(main())
