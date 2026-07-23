# Copyright 2026 Dimensional Inc.
"""Headless smoke entry point: ``python -m ...smoke``."""

from pathlib import Path

from .demo import run_demo
from .fake_backend import FakeGraspProposer


def main() -> int:
    run_demo(FakeGraspProposer(), Path("/tmp/graspgenx-ycb-demo.yaml"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
