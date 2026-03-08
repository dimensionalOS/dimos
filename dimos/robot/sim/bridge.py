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

"""NativeModule wrapper for the DimSim bridge subprocess.

Launches the DimSim bridge (Deno CLI) as a managed subprocess.  The bridge
publishes sensor data (odom, lidar, images) directly to LCM — no Python
decode/re-encode hop.  Python only handles lifecycle and TF (via DimSimTF).

Usage::

    from dimos.robot.sim.bridge import sim_bridge
    from dimos.robot.sim.tf_module import sim_tf
    from dimos.core.blueprints import autoconnect

    autoconnect(sim_bridge(), sim_tf(), some_consumer()).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

from dimos import spec
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.stream import In, Out
    from dimos.msgs.geometry_msgs import PoseStamped, Twist
    from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2

logger = setup_logger()

_DIMSIM_JSR = "jsr:@antim/dimsim"


def _find_deno() -> str:
    """Find the deno binary."""
    return shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")


def _find_local_cli() -> Path | None:
    """Find local DimSim/dimos-cli/cli.ts for development."""
    repo_root = Path(__file__).resolve().parents[4]
    candidate = repo_root / "DimSim" / "dimos-cli" / "cli.ts"
    return candidate if candidate.exists() else None


@dataclass(kw_only=True)
class DimSimBridgeConfig(NativeModuleConfig):
    """Configuration for the DimSim bridge subprocess."""

    # Set to deno binary — resolved in _resolve_paths().
    executable: str = "deno"
    build_command: str | None = None
    cwd: str | None = None

    scene: str = "apt"
    port: int = 8090
    local: bool = False  # Use local DimSim repo instead of installed CLI

    # These fields are handled via extra_args, not to_cli_args().
    cli_exclude: frozenset[str] = frozenset({"scene", "port", "local"})

    # Populated by _resolve_paths() — deno run args + dev subcommand + scene/port.
    extra_args: list[str] = field(default_factory=list)


class DimSimBridge(NativeModule, spec.Camera, spec.Pointcloud):
    """NativeModule that manages the DimSim bridge subprocess.

    The bridge (Deno process) handles Browser-LCM translation and publishes
    sensor data directly to LCM.  Ports declared here exist for blueprint
    wiring / autoconnect but data flows through LCM, not Python.
    """

    config: DimSimBridgeConfig
    default_config = DimSimBridgeConfig

    # Sensor outputs (bridge publishes these directly to LCM)
    odom: Out[PoseStamped]
    color_image: Out[Image]
    depth_image: Out[Image]
    lidar: Out[PointCloud2]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]

    # Control input (consumers publish cmd_vel to LCM, bridge reads it)
    cmd_vel: In[Twist]

    def _resolve_paths(self) -> None:
        """Resolve executable and build extra_args.

        Set DIMSIM_LOCAL=1 to use local DimSim repo instead of installed CLI.
        """
        dev_args = ["dev", "--scene", self.config.scene, "--port", str(self.config.port)]

        # DIMSIM_HEADLESS=1 → launch headless Chrome (no browser tab needed)
        # Uses CPU rendering (SwiftShader) by default — no GPU required for CI.
        # Set DIMSIM_RENDER=gpu for Metal/ANGLE on macOS.
        if os.environ.get("DIMSIM_HEADLESS", "").strip() in ("1", "true"):
            render = os.environ.get("DIMSIM_RENDER", "cpu").strip()
            dev_args.extend(["--headless", "--render", render])

        # Allow env var override: DIMSIM_LOCAL=1 dimos run sim-nav
        if os.environ.get("DIMSIM_LOCAL", "").strip() in ("1", "true"):
            self.config.local = True

        if self.config.local:
            cli_ts = _find_local_cli()
            if not cli_ts:
                raise FileNotFoundError(
                    "Local DimSim not found. Expected DimSim/dimos-cli/cli.ts "
                    "next to the dimos repo."
                )
            logger.info(f"Using local DimSim: {cli_ts}")
            self.config.executable = _find_deno()
            self.config.extra_args = [
                "run",
                "--allow-all",
                "--unstable-net",
                str(cli_ts),
                *dev_args,
            ]
            self.config.cwd = None
            return

        dimsim_path = shutil.which("dimsim") or str(Path.home() / ".deno" / "bin" / "dimsim")
        self.config.executable = dimsim_path
        self.config.extra_args = dev_args
        self.config.cwd = None

    def _maybe_build(self) -> None:
        """Ensure dimsim CLI, core assets, and scene are latest from S3."""
        if self.config.local:
            return  # Local dev — skip install

        import json
        import subprocess
        import urllib.request

        deno = _find_deno()
        scene = self.config.scene

        # Check installed CLI version against S3 registry
        dimsim = shutil.which("dimsim")
        installed_ver = None
        if dimsim:
            try:
                result = subprocess.run(
                    [dimsim, "--version"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                installed_ver = result.stdout.strip() if result.returncode == 0 else None
            except Exception:
                pass

        # Fetch registry version from S3 (tiny JSON, fast)
        registry_ver = None
        try:
            with urllib.request.urlopen(
                "https://dimsim-assets.s3.amazonaws.com/scenes.json", timeout=5
            ) as resp:
                registry_ver = json.loads(resp.read()).get("version")
        except Exception:
            pass

        if not dimsim or installed_ver != registry_ver:
            logger.info(
                f"Updating dimsim CLI: {installed_ver or 'not installed'}"
                f" → {registry_ver or 'latest'}",
            )
            subprocess.run(
                [deno, "install", "-gAf", "--unstable-net", _DIMSIM_JSR],
                check=True,
            )
            dimsim = shutil.which("dimsim")
            if not dimsim:
                raise FileNotFoundError("dimsim install failed — not found in PATH")
        else:
            logger.info(f"dimsim CLI up-to-date (v{installed_ver})")

        # setup/scene have version-aware caching (only downloads if version changed)
        logger.info("Checking core assets...")
        subprocess.run([dimsim, "setup"], check=True)

        logger.info(f"Checking scene '{scene}'...")
        subprocess.run([dimsim, "scene", "install", scene], check=True)

    def _collect_topics(self) -> dict[str, str]:
        """Bridge hardcodes LCM channel names — no topic args needed."""
        return {}


sim_bridge = DimSimBridge.blueprint

__all__ = ["DimSimBridge", "DimSimBridgeConfig", "sim_bridge"]
