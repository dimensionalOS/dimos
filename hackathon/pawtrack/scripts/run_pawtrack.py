"""Launch the PawTrack subject-tracking agentic blueprint.

PawTrack is not registered in DimOS ``all_blueprints``, so it ships its own
launcher. Modes:

- default (off-robot): ``McpServer`` + ``McpClient`` + the subject-tracking
  skill container. Enough to inspect tool discovery and call tools; no frames.
- ``--camera``: also run a webcam ``CameraModule`` and the Rerun viewer, so the
  whole VLM-acquire + EdgeTAM-track pipeline runs end to end with no robot --
  point a laptop camera at the subject and describe it.
- ``--source PATH``: same end-to-end test but fed from an image or video file
  instead of a webcam -- fully hardware-free (good for CI / a teammate).
- ``--robot``: the full ``unitree_go2_agentic`` stack + the tracker; the robot
  supplies the camera frames and viewer.

Describe the subject from the DimOS CLI (no robot needed):
    dimos mcp call track_subject --arg description="a person sitting on a chair"
    dimos agent-send "track the person sitting on the chair"   # via the agent

Run (in the venv), e.g. (the first two need no robot):
    PYTHONPATH=src python scripts/run_pawtrack.py --source room.mp4
    PYTHONPATH=src python scripts/run_pawtrack.py --camera
    PYTHONPATH=src python scripts/run_pawtrack.py --robot

Diagnostics: ``subject_status`` is a JSON LCM stream; ``debug_image`` is the
annotated camera frame shown in Rerun (green box when tracking, red while
searching).

Model defaults to ``openai:gpt-5.1`` (needs ``OPENAI_API_KEY``); override with
``PAWTRACK_MODEL`` (e.g. ``openai:deepseek-chat`` with ``OPENAI_BASE_URL``).
"""

from __future__ import annotations

import argparse
import importlib
import os

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.visualization.vis_module import vis_module

from pawtrack.image_source import FileImageSource
from pawtrack.skill_container import PawTrackSkillContainer
from pawtrack.system_prompt import PAWTRACK_PROMPT


def build_blueprint(
    robot: bool = False, camera: bool = False, source: str | None = None
):
    """Compose the PawTrack subject-tracking blueprint.

    Args:
        robot: Run on the full unitree_go2_agentic stack; the robot supplies
            camera frames and the viewer.
        camera: Off-robot -- add a webcam source and the Rerun viewer.
        source: Off-robot -- feed frames from this image/video file instead of
            a webcam (takes precedence over ``camera``).

    Returns:
        The composed blueprint ready for ModuleCoordinator.build.
    """
    container = PawTrackSkillContainer.blueprint()
    if robot:
        base = importlib.import_module(
            "dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic"
        ).unitree_go2_agentic
        # The agentic base already bundles the MCP server + agent, so we just
        # add our skill container (no second McpClient). The prompt-configured
        # agent lives in the registered `pawtrack-agentic` blueprint; this
        # ``--robot`` mode is a quick on-robot test of the standalone launcher.
        return autoconnect(base, container)
    model = os.environ.get("PAWTRACK_MODEL", "openai:gpt-5.1")
    agent = McpClient.blueprint(model=model, system_prompt=PAWTRACK_PROMPT)
    parts = [McpServer.blueprint(), container, agent]
    if source:
        parts.append(FileImageSource.blueprint(path=source))
        parts.append(vis_module(viewer_backend=global_config.viewer))
    elif camera:
        parts.append(CameraModule.blueprint())
        parts.append(vis_module(viewer_backend=global_config.viewer))
    return autoconnect(*parts)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the PawTrack agent.")
    parser.add_argument(
        "--robot",
        action="store_true",
        help="Run on the full Unitree Go2 agentic stack (venue mode).",
    )
    parser.add_argument(
        "--camera",
        action="store_true",
        help="Off-robot: add a webcam source + Rerun viewer for an "
        "end-to-end test with no robot.",
    )
    parser.add_argument(
        "--source",
        default=None,
        help="Off-robot: feed an image/video file instead of a webcam.",
    )
    args = parser.parse_args()
    ModuleCoordinator.build(
        build_blueprint(
            robot=args.robot, camera=args.camera, source=args.source
        )
    ).loop()


if __name__ == "__main__":
    main()
