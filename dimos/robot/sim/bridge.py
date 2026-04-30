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

Launches the DimSim bridge as a managed subprocess.  On first run, downloads
a compiled binary from GitHub Releases — no Deno runtime required.  The bridge
publishes sensor data (odom, lidar, images) directly to LCM — no Python
decode/re-encode hop.  Python only handles lifecycle and TF (via DimSimTF).

Usage::

    from dimos.robot.sim.bridge import sim_bridge
    from dimos.robot.sim.tf_module import sim_tf
    from dimos.core.blueprints import autoconnect

    autoconnect(sim_bridge(), sim_tf(), some_consumer()).build().loop()
"""

import os
from pathlib import Path
import shutil

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_GITHUB_REPO = "Antim-Labs/DimSim"
_RELEASES_API = f"https://api.github.com/repos/{_GITHUB_REPO}/releases/latest"


def _dimsim_bin() -> Path:
    """Path to the dimsim compiled binary."""
    return Path.home() / ".dimsim" / "bin" / "dimsim"


def _find_deno() -> str:
    """Find the deno binary (only needed for local dev mode)."""
    return shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")


def _find_local_cli() -> Path | None:
    """Find local DimSim/dimos-cli/cli.ts for development."""
    repo_root = Path(__file__).resolve().parents[4]
    candidate = repo_root / "DimSim" / "dimos-cli" / "cli.ts"
    return candidate if candidate.exists() else None


class DimSimBridgeConfig(NativeModuleConfig):
    """Configuration for the DimSim bridge subprocess."""

    # Resolved in _resolve_paths() — compiled binary or deno (local dev).
    executable: str = "dimsim"
    build_command: str | None = None
    cwd: str | None = None

    scene: str = "apt"
    port: int = 8090
    local: bool = False  # Use local DimSim repo instead of installed CLI

    # Sensor publish rates (ms). None = use DimSim defaults (images=200, lidar=200, odom=20).
    image_rate_ms: int | None = None
    lidar_rate_ms: int | None = None
    odom_rate_ms: int | None = None

    # Depth image toggle. Set False to skip depth publishing (saves bandwidth).
    enable_depth: bool = True

    # Camera FOV in degrees. None = use DimSim default (80).
    camera_fov: int | None = None

    # These fields are handled via extra_args, not to_cli_args().
    cli_exclude: frozenset[str] = frozenset(
        {
            "scene",
            "port",
            "local",
            "image_rate_ms",
            "lidar_rate_ms",
            "odom_rate_ms",
            "enable_depth",
            "camera_fov",
        }
    )

    # Populated by _resolve_paths() — deno run args + dev subcommand + scene/port.
    extra_args: list[str] = Field(default_factory=list)


class DimSimBridge(NativeModule, Camera, Pointcloud):
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

    @rpc
    def start(self) -> None:
        """Start the DimSim bridge subprocess.

        Set DIMSIM_CONNECT_ONLY=1 to skip subprocess launch — used when
        connecting to a shared dimsim server already started by another instance.
        """
        if os.environ.get("DIMSIM_CONNECT_ONLY", "").strip() in ("1", "true"):
            logger.info("DIMSIM_CONNECT_ONLY: skipping subprocess, connecting to existing server")
            return
        super().start()

    def _resolve_paths(self) -> None:
        """Resolve executable and build extra_args.

        Set DIMSIM_LOCAL=1 to use local DimSim repo instead of installed CLI.
        """
        # DIMSIM_SCENE env var overrides config (e.g. DIMSIM_SCENE=empty)
        scene = os.environ.get("DIMSIM_SCENE", "").strip() or self.config.scene
        dev_args = ["dev", "--scene", scene, "--port", str(self.config.port)]

        # Sensor publish rates (env var overrides config)
        image_rate = os.environ.get("DIMSIM_IMAGE_RATE", "").strip() or self.config.image_rate_ms
        lidar_rate = os.environ.get("DIMSIM_LIDAR_RATE", "").strip() or self.config.lidar_rate_ms
        odom_rate = os.environ.get("DIMSIM_ODOM_RATE", "").strip() or self.config.odom_rate_ms
        if image_rate is not None:
            dev_args.extend(["--image-rate", str(image_rate)])
        if lidar_rate is not None:
            dev_args.extend(["--lidar-rate", str(lidar_rate)])
        if odom_rate is not None:
            dev_args.extend(["--odom-rate", str(odom_rate)])

        # Depth image toggle
        no_depth = os.environ.get("DIMSIM_DISABLE_DEPTH", "").strip() in ("1", "true")
        if no_depth or not self.config.enable_depth:
            dev_args.append("--no-depth")

        # Camera FOV
        camera_fov = os.environ.get("DIMSIM_CAMERA_FOV", "").strip() or self.config.camera_fov
        if camera_fov is not None:
            dev_args.extend(["--camera-fov", str(camera_fov)])

        # DIMSIM_HEADLESS=1 → launch headless Chrome (no browser tab needed)
        # Uses CPU rendering (SwiftShader) by default — no GPU required for CI.
        # Set DIMSIM_RENDER=gpu for Metal/ANGLE on macOS.
        if os.environ.get("DIMSIM_HEADLESS", "").strip() in ("1", "true"):
            render = os.environ.get("DIMSIM_RENDER", "cpu").strip()
            dev_args.extend(["--headless", "--render", render])

        # DIMSIM_CHANNELS=N → multi-page mode (N browser pages for parallel evals)
        channels = os.environ.get("DIMSIM_CHANNELS", "").strip()
        if channels:
            dev_args.extend(["--channels", channels])

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

        # Prefer compiled binary over PATH (PATH may have stale deno-installed version)
        dimsim_bin = _dimsim_bin()
        dimsim_path = (
            str(dimsim_bin) if dimsim_bin.exists() else shutil.which("dimsim") or str(dimsim_bin)
        )
        self.config.executable = dimsim_path
        self.config.extra_args = dev_args
        self.config.cwd = None

    def _maybe_build(self) -> None:
        """Ensure dimsim binary, core assets, and scene are installed.

        Tries compiled binary from GitHub Releases first. Falls back to
        existing dimsim in PATH (e.g. installed via deno) for older releases
        that don't ship compiled binaries yet.
        """
        if self.config.local:
            return  # Local dev — skip install

        import json
        import platform
        import stat
        import subprocess
        import urllib.request

        scene = self.config.scene
        dimsim = _dimsim_bin()
        dimsim.parent.mkdir(parents=True, exist_ok=True)

        # Check installed version (compiled binary or PATH)
        dimsim_path = str(dimsim) if dimsim.exists() else shutil.which("dimsim")
        installed_ver = None
        if dimsim_path:
            try:
                result = subprocess.run(
                    [dimsim_path, "--version"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                installed_ver = result.stdout.strip() if result.returncode == 0 else None
            except Exception:
                pass

        # Fetch latest version from GitHub Releases
        latest_ver = None
        release_tag = None
        try:
            req = urllib.request.Request(
                _RELEASES_API,
                headers={"Accept": "application/vnd.github.v3+json"},
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read())
                release_tag = data["tag_name"]
                latest_ver = release_tag.lstrip("v")
        except Exception:
            pass

        if not dimsim_path or installed_ver != latest_ver:
            # Try downloading compiled binary from GitHub Releases
            downloaded = False
            if release_tag:
                system = platform.system().lower()
                machine = platform.machine().lower()
                if system == "darwin" and machine in ("arm64", "aarch64"):
                    binary_name = "dimsim-darwin-arm64"
                elif system == "darwin":
                    binary_name = "dimsim-darwin-x64"
                elif system == "linux" and machine in ("x86_64", "amd64"):
                    binary_name = "dimsim-linux-x64"
                else:
                    binary_name = None

                if binary_name:
                    url = (
                        f"https://github.com/{_GITHUB_REPO}/releases/download"
                        f"/{release_tag}/{binary_name}"
                    )
                    try:
                        logger.info(f"Downloading dimsim {latest_ver} for {system}/{machine}...")
                        urllib.request.urlretrieve(url, str(dimsim))
                        dimsim.chmod(
                            dimsim.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH
                        )
                        # macOS quarantines downloaded binaries — clear all xattrs
                        if system == "darwin":
                            subprocess.run(
                                ["xattr", "-c", str(dimsim)],
                                capture_output=True,
                            )
                        dimsim_path = str(dimsim)
                        downloaded = True
                        logger.info("dimsim binary installed.")
                    except Exception as exc:
                        logger.warning(
                            f"Compiled binary not available ({exc}), trying deno fallback..."
                        )

            # Fallback: install via deno if compiled binary not available
            if not downloaded and not dimsim_path:
                deno = shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")
                logger.info("Installing dimsim via deno (compiled binary not yet available)...")
                subprocess.run(
                    [deno, "install", "-gAf", "--reload", "--unstable-net", "jsr:@antim/dimsim"],
                    check=True,
                )
                dimsim_path = shutil.which("dimsim")
                if not dimsim_path:
                    dimsim_path = str(Path.home() / ".deno" / "bin" / "dimsim")
        else:
            logger.info(f"dimsim up-to-date (v{installed_ver})")

        if not dimsim_path:
            raise FileNotFoundError(
                "dimsim not found — install Deno and retry, or wait for next release with compiled binaries"
            )

        # Always update executable to the resolved path (may differ from _resolve_paths)
        self.config.executable = dimsim_path

        # Symlink to ~/.local/bin so `dimsim` is on PATH for eval authoring
        # Must happen BEFORE setup so that `dimsim eval list` etc. use the new binary
        local_bin = Path.home() / ".local" / "bin"
        local_bin.mkdir(parents=True, exist_ok=True)
        symlink = local_bin / "dimsim"
        try:
            target = Path(dimsim_path).resolve()
            if symlink.is_symlink() and symlink.resolve() != target:
                symlink.unlink()
            if not symlink.exists():
                symlink.symlink_to(target)
                logger.info(f"Symlinked dimsim → {symlink}")
        except OSError:
            pass  # no permission

        # setup/scene have version-aware caching (only downloads if version changed)
        logger.info("Checking core assets...")
        subprocess.run([dimsim_path, "setup"], check=True)

        logger.info(f"Checking scene '{scene}'...")
        subprocess.run([dimsim_path, "scene", "install", scene], check=True)

    def _collect_topics(self) -> dict[str, str]:
        """Bridge hardcodes LCM channel names — no topic args needed."""
        return {}


sim_bridge = DimSimBridge.blueprint

__all__ = ["DimSimBridge", "DimSimBridgeConfig", "sim_bridge"]
