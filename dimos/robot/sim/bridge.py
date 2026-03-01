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
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

from dimos import spec
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core import In, Out
    from dimos.msgs.geometry_msgs import PoseStamped, Twist
    from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2

logger = setup_logger()


def _find_cli_script() -> Path | None:
    """Auto-detect DimSim/dimos-cli/cli.ts relative to this repo."""
    repo_root = Path(__file__).resolve().parents[4]  # dimos/dimos/robot/sim -> repo
    candidate = repo_root / "DimSim" / "dimos-cli" / "cli.ts"
    return candidate if candidate.exists() else None


def _find_deno() -> str:
    """Find the deno binary."""
    return shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")


@dataclass(kw_only=True)
class DimSimBridgeConfig(NativeModuleConfig):
    """Configuration for the DimSim bridge subprocess."""

    # Set to deno binary — resolved in _resolve_paths().
    executable: str = "deno"
    build_command: str | None = None
    cwd: str | None = None

    scene: str = "apt"
    port: int = 8090
    cli_script: str | None = None

    # These fields are handled via extra_args, not to_cli_args().
    cli_exclude: frozenset[str] = frozenset({"scene", "port", "cli_script"})

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

        Prefers globally installed ``dimsim`` CLI (from JSR).  Falls back to
        running the local ``DimSim/dimos-cli/cli.ts`` via Deno for development.
        """
        dev_args = ["dev", "--scene", self.config.scene, "--port", str(self.config.port)]

        # 1. Prefer globally installed dimsim CLI (deno install jsr:@antim/dimsim)
        global_dimsim = shutil.which("dimsim")
        if global_dimsim:
            logger.info(f"Using global dimsim CLI: {global_dimsim}")
            self.config.executable = global_dimsim
            self.config.extra_args = dev_args
            self.config.cwd = None
            return

        # 2. Fall back to local deno + cli.ts (development mode)
        script = self.config.cli_script
        if script and Path(script).exists():
            cli_ts = str(Path(script).resolve())
        else:
            found = _find_cli_script()
            if found:
                cli_ts = str(found)
            else:
                raise FileNotFoundError(
                    "Cannot find DimSim. Install globally with:\n"
                    "  deno install -gAf --unstable-net jsr:@antim/dimsim\n"
                    "  dimsim setup && dimsim scene install apt"
                )

        self.config.executable = _find_deno()
        self.config.extra_args = [
            "run",
            "--allow-all",
            "--unstable-net",
            cli_ts,
            *dev_args,
        ]
        self.config.cwd = None

    def _maybe_build(self) -> None:
        """No build step needed for DimSim bridge."""

    def _collect_topics(self) -> dict[str, str]:
        """Bridge hardcodes LCM channel names — no topic args needed."""
        return {}


sim_bridge = DimSimBridge.blueprint

__all__ = ["DimSimBridge", "DimSimBridgeConfig", "sim_bridge"]
