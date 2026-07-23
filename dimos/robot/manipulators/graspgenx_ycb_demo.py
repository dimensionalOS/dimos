# Copyright 2026 Dimensional Inc.
"""Discoverable, no-MCP GraspGenX YCB demo deployment."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Literal

from pydantic import Field, model_validator

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import ModuleConfig, OneShotModule
from dimos.manipulation.graspgenx_demo.demo import (
    DEFAULT_ROI,
    ParentCompletion,
    run_demo,
)
from dimos.manipulation.graspgenx_demo.extractor import AxisAlignedROI
from dimos.manipulation.graspgenx_demo.visualization import (
    RerunLogger,
    launch_native_viewer,
    launch_web_viewer,
    recording_path_for_yaml,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXModule,
    SweepVolumeGripperConfig,
)


def deployment_config() -> GraspGenXConfig:
    """Build deployment settings without requiring the checkpoint at import time."""
    checkpoint = os.environ.get("DIMOS_GRASPGENX_CHECKPOINT", "/__missing_graspgenx_checkpoint__")
    return GraspGenXConfig(
        checkpoint_path=checkpoint,
        gripper=SweepVolumeGripperConfig(
            extents_open=(0.08, 0.045, 0.04),
            offset_open=(0.0, 0.0, 0.135),
            extents_half_open=(0.04, 0.045, 0.035),
            offset_half_open=(0.0, 0.0, 0.118),
            fingertip_depth=0.15,
            family="revolute_3f",
        ),
        max_candidates=100,
    )


DEPLOYMENT_CONFIG = deployment_config()


class DemoRunnerConfig(ModuleConfig):
    """One-shot controls; viewer is always taken from the active GlobalConfig."""

    output_path: str = os.environ.get("DIMOS_GRASPGENX_OUTPUT", "graspgenx-ycb-demo.yaml")
    deployment: GraspGenXConfig
    roi_minimum: tuple[float, float, float] = DEFAULT_ROI.minimum
    roi_maximum: tuple[float, float, float] = DEFAULT_ROI.maximum
    # This is the client-side timeout for the nested GraspGen RPC.  Inference
    # can initialize CUDA and retry model loading before producing a result.
    proposer_rpc_timeout: float = Field(default=600.0, gt=0.0)
    recording_path: str | None = None
    rerun_flush_timeout: float = Field(default=10.0, gt=0.0)
    native_window_backend: Literal["x11", "wayland", "auto"] = "x11"

    @model_validator(mode="after")
    def validate_roi(self) -> DemoRunnerConfig:
        import math

        bounds = (*self.roi_minimum, *self.roi_maximum)
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError("ROI bounds must be finite")
        if not math.isfinite(self.proposer_rpc_timeout):
            raise ValueError("proposer_rpc_timeout must be finite")
        if not math.isfinite(self.rerun_flush_timeout):
            raise ValueError("rerun_flush_timeout must be finite")
        if not all(
            minimum < maximum
            for minimum, maximum in zip(self.roi_minimum, self.roi_maximum, strict=True)
        ):
            raise ValueError("ROI minimum must be strictly less than maximum")
        return self


class GraspGenXYCBDemoRunner(OneShotModule):
    """One-shot sink that drives the complete offline scene-to-output path."""

    _proposer: GraspGenSpec
    config: DemoRunnerConfig  # type: ignore[assignment]

    def __init__(self, config: DemoRunnerConfig | None = None, **kwargs: Any) -> None:
        config_args = (
            config.model_dump(exclude={"rpc_transport", "tf_transport", "g"}) if config else {}
        )
        # ``config`` is created while the discoverable blueprint is imported.  Its
        # inherited ``g`` field therefore contains the import-time GlobalConfig,
        # not the instance supplied by the CLI/coordinator.  Keep the coordinator
        # value explicit when flattening the nested config; otherwise the frozen
        # blueprint value wins at the worker boundary.
        active_global_config = kwargs.get("g")
        if active_global_config is not None:
            config_args["g"] = active_global_config
        config_args.update(kwargs)
        super().__init__(**config_args)

    def _run_once(self) -> ParentCompletion:
        rerun_open = getattr(self.config.g, "rerun_open", "native")
        if isinstance(self.config.g, dict):
            rerun_open = self.config.g.get("rerun_open", "native")
        if rerun_open not in {"none", "native", "web", "both"}:
            raise ValueError(
                f"unsupported rerun_open={rerun_open!r}; choose none, native, web, or both"
            )
        final_path = recording_path_for_yaml(
            Path(self.config.output_path),
            Path(self.config.recording_path) if self.config.recording_path else None,
        )
        logger: RerunLogger | None = None
        viewer = (
            self.config.g.get("viewer", "rerun")
            if isinstance(self.config.g, dict)
            else self.config.g.viewer
        )
        if viewer == "rerun":
            logger = RerunLogger(final_path, flush_timeout=self.config.rerun_flush_timeout)
        # ``rpc_timeout`` is consumed by RpcCall and is not sent as a method
        # argument.  Keep this at the deployment seam so cold inference has a
        # validated, explicit timeout without adding retries or another
        # transport.
        proposer: Any = self._proposer
        try:
            run_demo(
                proposer,
                Path(self.config.output_path),
                roi=AxisAlignedROI(self.config.roi_minimum, self.config.roi_maximum),
                checkpoint=self.config.deployment.checkpoint_path,
                device=os.environ.get("DIMOS_GRASPGENX_DEVICE", "cuda"),
                cuda=None,
                tcp_calibration=self.config.deployment.grasp_frame_to_tcp,
                gripper=self.config.deployment.gripper,
                logger=logger,
                proposer_rpc_timeout=self.config.proposer_rpc_timeout,
            )
            recording = logger.finalize() if logger is not None else None
            return ParentCompletion(
                str(recording) if recording is not None else None,
                str(rerun_open),
                self.config.native_window_backend,
            )
        finally:
            if logger is not None:
                logger.abort()

    @classmethod
    def _complete_in_parent(cls, result: ParentCompletion) -> None:
        """Open only the finalized recording, after the CLI has cleaned workers."""
        if not result.recording_path:
            return
        recording = Path(result.recording_path).expanduser().resolve()
        if not recording.is_file():
            print(
                f"graspgenx-ycb-demo recording missing; open manually: rerun {recording}",
                flush=True,
            )
            return
        if result.rerun_open == "none":
            return
        if result.rerun_open in ("native", "both"):
            launch_native_viewer(recording, window_backend=result.native_window_backend)
        if result.rerun_open in ("web", "both"):
            launch_web_viewer(recording)


graspgenx_ycb_demo = autoconnect(
    GraspGenXModule.blueprint(
        **DEPLOYMENT_CONFIG.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
    GraspGenXYCBDemoRunner.blueprint(
        **DemoRunnerConfig(deployment=DEPLOYMENT_CONFIG).model_dump(
            exclude={"rpc_transport", "tf_transport", "g"}
        )
    ),
)
