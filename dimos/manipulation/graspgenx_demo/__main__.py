# Copyright 2026 Dimensional Inc.
"""Contributor entry point for the direct-process GraspGenX YCB demo."""

import argparse
from collections.abc import Callable, Sequence
import os
from pathlib import Path
from typing import Any

from dimos.manipulation.grasping.grasp_gen_x import GraspGenXConfig, GraspGenXModule

from .demo import run_contributor_demo


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(os.environ.get("DIMOS_GRASPGENX_OUTPUT", "graspgenx-ycb-demo.yaml")),
    )
    parser.add_argument("--recording-path", type=Path, default=None)
    parser.add_argument(
        "--viewer", choices=("rerun", "none"), default=os.environ.get("DIMOS_VIEWER", "rerun")
    )
    parser.add_argument(
        "--rerun-open",
        choices=("native", "web", "both", "none"),
        default=os.environ.get("DIMOS_RERUN_OPEN", "native"),
    )
    parser.add_argument(
        "--native-window-backend",
        choices=("x11", "wayland", "auto"),
        default=os.environ.get("DIMOS_GRASPGENX_NATIVE_WINDOW_BACKEND", "x11"),
    )
    parser.add_argument("--flush-timeout", type=float, default=10.0)
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    module_factory: Callable[[GraspGenXConfig], GraspGenXModule] = GraspGenXModule,
    runner: Callable[..., Any] = run_contributor_demo,
) -> int:
    args = _parser().parse_args(argv)
    result = runner(
        output_path=args.output,
        recording_path=args.recording_path,
        viewer=args.viewer,
        rerun_open=args.rerun_open,
        native_window_backend=args.native_window_backend,
        flush_timeout=args.flush_timeout,
        module_factory=module_factory,
    )
    print(
        f"graspgenx-ycb-demo complete candidates={result.candidate_count} "
        f"yaml={result.output_path} recording={result.recording_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
