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

"""DimSim bridge module with Unity-compatible interface.

Launches the DimSim browser-based simulator as a managed subprocess,
subscribes to its LCM sensor output, and exposes the same stream
interface as UnityBridgeModule — making it a drop-in replacement for
nav/planning tests on macOS and ARM Linux.

Data flow:
    DimSim subprocess → LCM (odom, lidar, images) → DimSimBridge Out ports
    DimSimBridge In[cmd_vel] → LCM → DimSim subprocess

Uses --topic-remap (jeff-hykin/DimSim fork) to namespace LCM channels,
enabling multiple simultaneous instances.

Usage::

    from dimos.robot.sim.bridge import sim_bridge
    from dimos.core.coordination.blueprints import autoconnect

    autoconnect(sim_bridge(scene="apt"), some_consumer()).build().loop()
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import threading
import time
from typing import Any

import lcm as lcmlib
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_GITHUB_REPO = "jeff-hykin/DimSim"
_RELEASES_API = f"https://api.github.com/repos/{_GITHUB_REPO}/releases/latest"

# Camera defaults (DimSim: 640x288, configurable FOV)
_CAM_W = 640
_CAM_H = 288
_DEFAULT_FOV_DEG = int(os.environ.get("DIMSIM_CAMERA_FOV", "46"))


def _dimsim_bin() -> Path:
    return Path.home() / ".dimsim" / "bin" / "dimsim"


def _find_deno() -> str:
    return shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")


def _find_local_cli() -> Path | None:
    """Find local DimSim/dimos-cli/cli.ts for development."""
    candidate = Path.home() / "repos" / "DimSim" / "dimos-cli" / "cli.ts"
    if candidate.exists():
        return candidate
    repo_root = Path(__file__).resolve().parents[4]
    candidate = repo_root / "DimSim" / "dimos-cli" / "cli.ts"
    return candidate if candidate.exists() else None


def _make_camera_info(fov_deg: int | None = None) -> CameraInfo:
    """Build CameraInfo for DimSim's virtual camera."""
    fov = fov_deg or _DEFAULT_FOV_DEG
    fx = (_CAM_W / 2) / math.tan(math.radians(fov / 2))
    fy = fx
    cx, cy = _CAM_W / 2.0, _CAM_H / 2.0

    return CameraInfo(
        frame_id="camera_optical",
        height=_CAM_H,
        width=_CAM_W,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


class DimSimBridgeConfig(ModuleConfig):
    """Configuration for the DimSim bridge."""

    scene: str = "apt"
    port: int = 8090
    local: bool = False

    # Sensor publish rates (ms). None = DimSim defaults.
    image_rate_ms: int | None = None
    lidar_rate_ms: int | None = None
    odom_rate_ms: int | None = None

    enable_depth: bool = True
    camera_fov: int | None = None

    # Sim parameters (matching UnityBridgeConfig interface)
    vehicle_height: float = 0.75

    # Topic namespace prefix for multi-instance isolation
    topic_prefix: str = ""

    # LCM URL override
    lcm_url: str = ""


class DimSimBridge(Module):
    """Browser-based 3D simulator with Unity-compatible stream interface.

    Ports (matching UnityBridgeModule):
        cmd_vel (In[Twist]): Velocity commands.
        terrain_map (In[PointCloud2]): Terrain for Z adjustment (accepted for compat).
        odometry (Out[Odometry]): Vehicle state (converted from PoseStamped).
        registered_scan (Out[PointCloud2]): Lidar pointcloud.
        color_image (Out[Image]): RGB camera.
        camera_info (Out[CameraInfo]): Camera intrinsics.
    """

    config: DimSimBridgeConfig

    # Unity-compatible ports
    cmd_vel: In[Twist]
    terrain_map: In[PointCloud2]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    @staticmethod
    def rerun_blueprint() -> Any:
        """3D world view for DimSim visualization."""
        import rerun.blueprint as rrb

        return rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial3DView(
                    origin="world",
                    name="3D",
                    eye_controls=rrb.EyeControls3D(
                        position=(0.0, 0.0, 20.0),
                        look_target=(0.0, 0.0, 0.0),
                        eye_up=(0.0, 0.0, 1.0),
                    ),
                ),
            ),
            collapse_panels=True,
        )

    @staticmethod
    def rerun_static_pinhole(rr: Any) -> list[Any]:
        """Static Pinhole + Transform3D for the DimSim camera."""
        fov = _DEFAULT_FOV_DEG
        fx = (_CAM_W / 2) / math.tan(math.radians(fov / 2))
        return [
            rr.Pinhole(
                resolution=[_CAM_W, _CAM_H],
                focal_length=[fx, fx],
                principal_point=[_CAM_W / 2, _CAM_H / 2],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
            rr.Transform3D(
                parent_frame="tf#/sensor",
                translation=[0.3, 0.0, 0.0],
                rotation=rr.Quaternion(xyzw=[0.5, -0.5, 0.5, -0.5]),
            ),
        ]

    @staticmethod
    def rerun_suppress_camera_info(_: Any) -> None:
        return None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = threading.Event()
        self._process: subprocess.Popen | None = None  # type: ignore[type-arg]
        self._lcm: lcmlib.LCM | None = None
        self._lcm_thread: threading.Thread | None = None
        self._caminfo_thread: threading.Thread | None = None

        # For velocity estimation (PoseStamped → Odometry)
        self._prev_odom_time: float | None = None
        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_yaw = 0.0

        # Build remapped topic names
        p = self.config.topic_prefix
        self._topic_odom = f"{p}/odom#geometry_msgs.PoseStamped"
        self._topic_lidar = f"{p}/lidar#sensor_msgs.PointCloud2"
        self._topic_color = f"{p}/color_image#sensor_msgs.Image"
        self._topic_cmd_vel = f"{p}/cmd_vel#geometry_msgs.Twist"

    @rpc
    def start(self) -> None:
        if os.environ.get("DIMSIM_CONNECT_ONLY", "").strip() in ("1", "true"):
            logger.info("DIMSIM_CONNECT_ONLY: skipping subprocess, connecting to existing server")
        else:
            self._ensure_installed()
            threading.Thread(target=self._launch_subprocess, daemon=True).start()

        super().start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

        # Start LCM listener for sensor data from subprocess
        lcm_url = self.config.lcm_url or os.environ.get(
            "LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0"
        )
        self._lcm = lcmlib.LCM(lcm_url)
        self._lcm.subscribe(self._topic_odom, self._on_lcm_odom)
        self._lcm.subscribe(self._topic_lidar, self._on_lcm_lidar)
        self._lcm.subscribe(self._topic_color, self._on_lcm_color_image)

        self._running.set()
        self._lcm_thread = threading.Thread(target=self._lcm_loop, daemon=True)
        self._lcm_thread.start()

        # Publish camera_info immediately + at 1 Hz
        self.camera_info.publish(_make_camera_info(self.config.camera_fov))
        self._caminfo_thread = threading.Thread(target=self._caminfo_loop, daemon=True)
        self._caminfo_thread.start()

        logger.info("DimSimBridge started")

    @rpc
    def stop(self) -> None:
        self._running.clear()
        if self._lcm_thread:
            self._lcm_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._caminfo_thread:
            self._caminfo_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        proc = self._process
        self._process = None
        if proc is not None and proc.poll() is None:
            logger.info(f"Stopping DimSim (pid={proc.pid})")
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning(f"DimSim pid={proc.pid} did not exit, killing")
                proc.kill()
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command (UnityBridgeModule compat)."""
        if duration > 0:
            stop = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self._publish_cmd_vel(twist)
                time.sleep(0.05)
            self._publish_cmd_vel(stop)
        else:
            self._publish_cmd_vel(twist)
        return True

    # -- Subprocess management -------------------------------------------------

    def _build_cli_args(self) -> list[str]:
        cfg = self.config
        scene = os.environ.get("DIMSIM_SCENE", "").strip() or cfg.scene
        args = ["dev", "--scene", scene, "--port", str(cfg.port)]

        image_rate = os.environ.get("DIMSIM_IMAGE_RATE", "").strip() or cfg.image_rate_ms
        lidar_rate = os.environ.get("DIMSIM_LIDAR_RATE", "").strip() or cfg.lidar_rate_ms
        odom_rate = os.environ.get("DIMSIM_ODOM_RATE", "").strip() or cfg.odom_rate_ms
        if image_rate is not None:
            args.extend(["--image-rate", str(image_rate)])
        if lidar_rate is not None:
            args.extend(["--lidar-rate", str(lidar_rate)])
        if odom_rate is not None:
            args.extend(["--odom-rate", str(odom_rate)])

        no_depth = os.environ.get("DIMSIM_DISABLE_DEPTH", "").strip() in ("1", "true")
        if no_depth or not cfg.enable_depth:
            args.append("--no-depth")

        camera_fov = os.environ.get("DIMSIM_CAMERA_FOV", "").strip() or cfg.camera_fov
        if camera_fov is not None:
            args.extend(["--camera-fov", str(camera_fov)])

        if os.environ.get("DIMSIM_HEADLESS", "").strip() in ("1", "true"):
            render = os.environ.get("DIMSIM_RENDER", "cpu").strip()
            args.extend(["--headless", "--render", render])

        channels = os.environ.get("DIMSIM_CHANNELS", "").strip()
        if channels:
            args.extend(["--channels", channels])

        # Topic remapping
        if cfg.topic_prefix:
            remap_pairs = [
                f"/odom={cfg.topic_prefix}/odom",
                f"/lidar={cfg.topic_prefix}/lidar",
                f"/color_image={cfg.topic_prefix}/color_image",
                f"/depth_image={cfg.topic_prefix}/depth_image",
                f"/cmd_vel={cfg.topic_prefix}/cmd_vel",
                f"/camera_info={cfg.topic_prefix}/camera_info",
            ]
            args.extend(["--topic-remap", ",".join(remap_pairs)])

        return args

    def _resolve_executable(self) -> tuple[str, list[str]]:
        use_local = self.config.local or os.environ.get("DIMSIM_LOCAL", "").strip() in ("1", "true")

        if use_local:
            cli_ts = _find_local_cli()
            if not cli_ts:
                raise FileNotFoundError(
                    "Local DimSim not found. Expected DimSim/dimos-cli/cli.ts"
                )
            logger.info(f"Using local DimSim: {cli_ts}")
            return _find_deno(), ["run", "--allow-all", "--unstable-net", str(cli_ts)]

        dimsim = _dimsim_bin()
        if dimsim.exists():
            return str(dimsim), []

        path_dimsim = shutil.which("dimsim")
        if path_dimsim:
            return path_dimsim, []

        raise FileNotFoundError(
            "dimsim not found — run `dimsim setup` or install via deno"
        )

    def _ensure_installed(self) -> None:
        """Download binary + setup + scene install if needed."""
        if self.config.local or os.environ.get("DIMSIM_LOCAL", "").strip() in ("1", "true"):
            return

        import json
        import platform
        import stat
        import urllib.request

        scene = os.environ.get("DIMSIM_SCENE", "").strip() or self.config.scene
        dimsim = _dimsim_bin()
        dimsim.parent.mkdir(parents=True, exist_ok=True)

        dimsim_path = str(dimsim) if dimsim.exists() else shutil.which("dimsim")
        installed_ver = None
        if dimsim_path:
            try:
                result = subprocess.run(
                    [dimsim_path, "--version"],
                    capture_output=True, text=True, timeout=5,
                )
                installed_ver = result.stdout.strip() if result.returncode == 0 else None
            except Exception:
                pass

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
                        if system == "darwin":
                            subprocess.run(["xattr", "-c", str(dimsim)], capture_output=True)
                        dimsim_path = str(dimsim)
                        downloaded = True
                        logger.info("dimsim binary installed.")
                    except Exception as exc:
                        logger.warning(f"Binary download failed ({exc}), trying deno fallback...")

            if not downloaded and not dimsim_path:
                deno = shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")
                logger.info("Installing dimsim via deno...")
                subprocess.run(
                    [deno, "install", "-gAf", "--reload", "--unstable-net", "jsr:@antim/dimsim"],
                    check=True,
                )
                dimsim_path = shutil.which("dimsim") or str(Path.home() / ".deno" / "bin" / "dimsim")
        else:
            logger.info(f"dimsim up-to-date (v{installed_ver})")

        if not dimsim_path:
            raise FileNotFoundError("dimsim not found")

        # Symlink to ~/.local/bin
        local_bin = Path.home() / ".local" / "bin"
        local_bin.mkdir(parents=True, exist_ok=True)
        symlink = local_bin / "dimsim"
        try:
            target = Path(dimsim_path).resolve()
            if symlink.is_symlink() and symlink.resolve() != target:
                symlink.unlink()
            if not symlink.exists():
                symlink.symlink_to(target)
        except OSError:
            pass

        logger.info("Checking core assets...")
        subprocess.run([dimsim_path, "setup"], check=True)
        logger.info(f"Checking scene '{scene}'...")
        subprocess.run([dimsim_path, "scene", "install", scene], check=True)

    def _launch_subprocess(self) -> None:
        try:
            exe, prefix_args = self._resolve_executable()
        except FileNotFoundError as e:
            logger.error(str(e))
            return

        cli_args = self._build_cli_args()
        cmd = [exe, *prefix_args, *cli_args]
        logger.info(f"Launching DimSim: {' '.join(cmd)}")

        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={**os.environ},
        )

        def _drain() -> None:
            proc = self._process
            if proc and proc.stderr:
                for line in proc.stderr:
                    if self._running.is_set():
                        logger.debug(f"[dimsim] {line.decode(errors='replace').rstrip()}")

        threading.Thread(target=_drain, daemon=True).start()

    # -- LCM listener ----------------------------------------------------------

    def _lcm_loop(self) -> None:
        while self._running.is_set():
            try:
                if self._lcm:
                    self._lcm.handle_timeout(100)
            except Exception:
                pass

    def _on_lcm_odom(self, channel: str, data: bytes) -> None:
        """Convert PoseStamped → Odometry and publish + TF."""
        try:
            ps = PoseStamped.lcm_decode(data)
        except Exception:
            return

        now = time.time()
        x, y, z = ps.x, ps.y, ps.z
        qx, qy, qz, qw = ps.qx, ps.qy, ps.qz, ps.qw

        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Estimate body-frame velocity from position deltas
        vx = vy = vyaw = 0.0
        if self._prev_odom_time is not None:
            dt = now - self._prev_odom_time
            if dt > 0.001:
                dx = x - self._prev_x
                dy = y - self._prev_y
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                vx = (dx * cos_yaw + dy * sin_yaw) / dt
                vy = (-dx * sin_yaw + dy * cos_yaw) / dt
                dyaw = yaw - self._prev_yaw
                while dyaw > math.pi:
                    dyaw -= 2 * math.pi
                while dyaw < -math.pi:
                    dyaw += 2 * math.pi
                vyaw = dyaw / dt

        self._prev_odom_time = now
        self._prev_x = x
        self._prev_y = y
        self._prev_yaw = yaw

        self.odometry.publish(
            Odometry(
                ts=ps.ts, frame_id="world", child_frame_id="base_link",
                x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw,
                vx=vx, vy=vy, vz=0.0, wx=0.0, wy=0.0, wz=vyaw,
            )
        )

        # TF: world -> base_link -> sensor
        self.tf.publish(
            Transform(
                ts=ps.ts, parent_frame_id="world", child_frame_id="base_link",
                translation=Vector3(x=x, y=y, z=z),
                rotation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            )
        )
        self.tf.publish(
            Transform(
                ts=ps.ts, parent_frame_id="base_link", child_frame_id="sensor",
                translation=Vector3(x=0.3, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    def _on_lcm_lidar(self, channel: str, data: bytes) -> None:
        try:
            self.registered_scan.publish(PointCloud2.lcm_decode(data))
        except Exception as e:
            logger.debug(f"Lidar decode error: {e}")

    def _on_lcm_color_image(self, channel: str, data: bytes) -> None:
        try:
            self.color_image.publish(Image.lcm_decode(data))
        except Exception as e:
            logger.debug(f"Image decode error: {e}")

    # -- cmd_vel → LCM ----------------------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._publish_cmd_vel(twist)

    def _publish_cmd_vel(self, twist: Twist) -> None:
        if self._lcm:
            try:
                self._lcm.publish(self._topic_cmd_vel, twist.lcm_encode())
            except Exception:
                pass

    # -- Camera info -------------------------------------------------------------

    def _caminfo_loop(self) -> None:
        while self._running.is_set():
            self.camera_info.publish(_make_camera_info(self.config.camera_fov))
            time.sleep(1.0)


sim_bridge = DimSimBridge.blueprint

__all__ = ["DimSimBridge", "DimSimBridgeConfig", "sim_bridge"]
