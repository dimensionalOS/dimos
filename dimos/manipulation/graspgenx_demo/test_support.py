# Copyright 2026 Dimensional Inc.
"""Forkserver-picklable fake deployment helpers for focused tests."""

import os
from pathlib import Path
import time

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.robot.manipulators.graspgenx_ycb_demo import (
    DemoRunnerConfig,
    GraspGenXYCBDemoRunner,
    deployment_config,
)


def fake_factory(config: object) -> object:
    return object()


def fake_inference(_sampler: object, _points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    delay = float(os.environ.get("DIMOS_GRASPGENX_FAKE_DELAY", "0"))
    if delay:
        time.sleep(delay)
    if os.environ.get("DIMOS_GRASPGENX_FAKE_FAIL") == "1":
        raise RuntimeError("fake GraspGen inference failed")
    poses = np.repeat(np.eye(4, dtype=np.float32)[None, ...], 3, axis=0)
    poses[:, :3, 3] = np.asarray([0.25, 0.18, 0.16])
    return poses, np.asarray([0.91, 0.73, 0.52], dtype=np.float32)


def fake_blueprint(output: Path | str, *, proposer_rpc_timeout: float = 600.0):
    config = deployment_config()
    return autoconnect(
        GraspGenXModule.blueprint(config=config, factory=fake_factory, inference=fake_inference),
        GraspGenXYCBDemoRunner.blueprint(
            config=DemoRunnerConfig(
                deployment=config,
                output_path=str(output),
                proposer_rpc_timeout=proposer_rpc_timeout,
                g={"viewer": "none"},
            )
        ),
    )
