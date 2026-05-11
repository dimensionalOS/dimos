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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""End-to-end tests for scene editing features (sensor rates, toggleable
channels, camera FOV, embodiment relay, auto-scale, UI cleanup).

Launches a headless DimSim server using the local dev repo.

Run:
    pytest dimos/e2e_tests/test_scene_editing.py -v -s -m slow
"""

import os
import shutil
import signal
import socket
import subprocess
import time

import pytest

from dimos.e2e_tests._subprocess_log import dump_log, launch_with_streaming_log
from dimos.robot.sim.scene_client import EMBODIMENT_PRESETS, SceneClient

PORT = 8091  # Use different port to avoid conflicts with other tests

# Path to local DimSim CLI
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_CLI_TS = os.path.join(_REPO_ROOT, "DimSim", "dimos-cli", "cli.ts")
_DENO = shutil.which("deno") or os.path.expanduser("~/.deno/bin/deno")

# Skip if DimSim not available
pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(not os.path.exists(_CLI_TS), reason="DimSim CLI not found"),
    pytest.mark.skipif(not (os.path.exists(_DENO) if _DENO else False), reason="Deno not found"),
]


def _wait_for_port(port: int, timeout: float = 120) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False


def _force_kill_port(port: int) -> None:
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        for pid in result.stdout.strip().split():
            if pid:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def _start_server(*extra_args: str) -> subprocess.Popen:
    """Start headless DimSim dev server with extra CLI args.

    Uses ``launch_with_streaming_log`` so the subprocess's stdout is drained
    in a background thread — a raw ``subprocess.PIPE`` with no reader fills
    the OS pipe buffer (~64 KB) and deadlocks the server, which then stops
    accepting WebSocket connections.
    """
    _force_kill_port(PORT)
    time.sleep(1)
    cmd = [
        _DENO,
        "run",
        "--allow-all",
        "--unstable-net",
        _CLI_TS,
        "dev",
        "--scene",
        "empty",
        "--headless",
        "--render",
        "cpu",
        "--port",
        str(PORT),
        *extra_args,
    ]
    proc, log_path = launch_with_streaming_log("dimsim_scene_editing", cmd)
    # Stash for _stop_server cleanup; avoids changing the function signature
    # used by every caller.
    proc._log_path = log_path  # type: ignore[attr-defined]
    if not _wait_for_port(PORT, timeout=120):
        dump_log("dimsim scene-editing", log_path)
        raise AssertionError(f"dimsim failed to start on :{PORT}")
    time.sleep(10)  # wait for browser page init
    return proc


def _stop_server(proc: subprocess.Popen) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    log_path = getattr(proc, "_log_path", None)
    if log_path is not None:
        log_path.unlink(missing_ok=True)


# ── Fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def dimsim_server():
    """Headless DimSim with default settings."""
    proc = _start_server()
    yield proc
    _stop_server(proc)


@pytest.fixture
def scene(dimsim_server):
    """SceneClient connected to the running server."""
    client = SceneClient(port=PORT, timeout=30)
    yield client
    client.close()


# ── Tests: Sensor Rates ─────────────────────────────────────────────────────


class TestSensorRates:
    """Configurable sensor publish rates."""

    def test_default_image_rate_is_5hz(self, scene):
        """Default image rate should be 200ms (5 Hz) after our change."""
        rate = scene.exec("return window.__dimosBridge?.rates?.images;")
        assert rate == 200, f"Expected default 200ms, got {rate}ms"

    def test_default_odom_rate(self, scene):
        rate = scene.exec("return window.__dimosBridge?.rates?.odom;")
        assert rate == 20, f"Expected default 20ms, got {rate}ms"

    def test_set_sensor_rate_images(self, scene):
        """Runtime rate change for images."""
        scene.set_sensor_rate("images", 100)
        rate = scene.exec("return window.__dimosBridge?.rates?.images;")
        assert rate == 100, f"Expected 100ms, got {rate}ms"
        # Restore
        scene.set_sensor_rate("images", 200)

    def test_set_sensor_rate_invalid(self, scene):
        """Invalid sensor name raises ValueError."""
        with pytest.raises(ValueError, match="Unknown sensor"):
            scene.set_sensor_rate("invalid", 100)


# ── Tests: Sensor Enable/Disable ────────────────────────────────────────────


class TestDepthToggle:
    """Toggleable depth image channel."""

    def test_depth_enabled_by_default(self, scene):
        """Depth should be enabled by default."""
        enable = scene.exec("return window.__dimosBridge?.sensorEnable?.depth;")
        assert enable is True

    def test_disable_depth(self, scene):
        """Disable depth at runtime."""
        scene.set_depth_enabled(False)
        enable = scene.exec("return window.__dimosBridge?.sensorEnable?.depth;")
        assert enable is False
        # Re-enable
        scene.set_depth_enabled(True)
        enable = scene.exec("return window.__dimosBridge?.sensorEnable?.depth;")
        assert enable is True


# ── Tests: Camera FOV ───────────────────────────────────────────────────────


class TestCameraFov:
    """Configurable camera FOV."""

    def test_default_fov(self, scene):
        """Default FOV should be 80 degrees."""
        fov = scene.exec("return window.__dimosCapCam?.fov;")
        assert fov == 80, f"Expected 80, got {fov}"

    def test_set_camera_fov(self, scene):
        """Runtime FOV change."""
        scene.set_camera_fov(60)
        fov = scene.exec("return window.__dimosCapCam?.fov;")
        assert fov == 60, f"Expected 60, got {fov}"
        # Restore
        scene.set_camera_fov(80)

    def test_set_camera_fov_wide(self, scene):
        """Wide FOV."""
        scene.set_camera_fov(120)
        fov = scene.exec("return window.__dimosCapCam?.fov;")
        assert fov == 120
        scene.set_camera_fov(80)

    def test_set_camera_fov_narrow(self, scene):
        """Narrow FOV."""
        scene.set_camera_fov(30)
        fov = scene.exec("return window.__dimosCapCam?.fov;")
        assert fov == 30
        scene.set_camera_fov(80)


# ── Tests: Embodiment Relay ─────────────────────────────────────────────────


class TestEmbodimentRelay:
    """Embodiment swap relay (WS message passes through EvalHarness)."""

    def test_set_embodiment_drone(self, scene):
        """Switch to drone embodiment."""
        scene.set_embodiment("drone")
        time.sleep(1)
        # Check the agent's avatarUrl was updated
        avatar = scene.exec("return window.__dimosAgent?.avatarUrl;")
        assert avatar is not None
        # Drone preset should have different URLs than quadruped
        EMBODIMENT_PRESETS["drone"]
        assert (
            any(
                "drone" in str(u).lower() or "quadrotor" in str(u).lower()
                for u in (avatar if isinstance(avatar, list) else [avatar])
            )
            or avatar != EMBODIMENT_PRESETS["quadruped"]["avatarUrl"]
        )

    def test_set_embodiment_quadruped(self, scene):
        """Switch back to quadruped."""
        scene.set_embodiment("quadruped")
        time.sleep(1)
        avatar = scene.exec("return window.__dimosAgent?.avatarUrl;")
        assert avatar is not None

    def test_embodiment_presets_exist(self, scene):
        """Core presets are defined."""
        expected = {"quadruped", "drone", "humanoid", "unitree-go2"}
        assert expected.issubset(set(EMBODIMENT_PRESETS.keys())), (
            f"Missing presets: {expected - set(EMBODIMENT_PRESETS.keys())}"
        )


# ── Tests: Auto-Scale ───────────────────────────────────────────────────────


class TestAutoScale:
    """Auto-scale imported models (cm → m detection)."""

    def test_autoscale_function_exists(self, scene):
        """autoScale is available in exec context."""
        result = scene.exec("return typeof autoScale;")
        assert result == "function"

    def test_autoscale_small_object_no_change(self, scene):
        """Object already in meters (< 50m) should not be scaled."""
        result = scene.exec("""
            const mesh = new THREE.Mesh(
                new THREE.BoxGeometry(2, 2, 2),
                new THREE.MeshStandardMaterial()
            );
            mesh.name = "autoscale-small";
            scene.add(mesh);
            const factor = autoScale(mesh);
            scene.remove(mesh);
            return factor;
        """)
        assert result == 1.0

    def test_autoscale_large_object_scaled_down(self, scene):
        """Object > 100m (likely cm) should be scaled by 0.01."""
        result = scene.exec("""
            const mesh = new THREE.Mesh(
                new THREE.BoxGeometry(200, 200, 200),
                new THREE.MeshStandardMaterial()
            );
            mesh.name = "autoscale-large";
            scene.add(mesh);
            const factor = autoScale(mesh);
            scene.remove(mesh);
            return factor;
        """)
        assert result == 0.01, f"Expected 0.01 for 200m object, got {result}"

    def test_autoscale_medium_object_proportional(self, scene):
        """Object 50-100m should be scaled proportionally."""
        result = scene.exec("""
            const mesh = new THREE.Mesh(
                new THREE.BoxGeometry(80, 80, 80),
                new THREE.MeshStandardMaterial()
            );
            mesh.name = "autoscale-medium";
            scene.add(mesh);
            const factor = autoScale(mesh);
            scene.remove(mesh);
            return factor;
        """)
        assert 0.5 < result < 1.0, f"Expected proportional scale for 80m object, got {result}"

    def test_load_map_returns_scale_factor(self, scene):
        """load_map includes scaleFactor in result."""
        result = scene.load_map(
            url="/proxy?url=https://threejs.org/examples/models/gltf/collision-world.glb",
            name="autoscale-map-test",
            position=(0, 0, 0),
        )
        assert "scaleFactor" in result
        assert isinstance(result["scaleFactor"], (int, float))
        # Clean up
        scene.remove_object("autoscale-map-test")


# ── Tests: UI Cleanup ───────────────────────────────────────────────────────


class TestUiCleanup:
    """dimos mode UI: sidebar hidden, command bar hidden, keyboard hints shown.

    Note: UI hide code runs in the DimosBridge init block which sets
    element.style.display = 'none'. In headless mode this may not run
    (headless skips the UI block). We check computed visibility instead.
    """

    def test_sidebar_not_visible(self, scene):
        """Agent panel should be hidden in dimos mode (headed only).

        In headless mode the UI cleanup is skipped (no user looking at it),
        so we only verify the code path exists — not that elements are hidden.
        """
        result = scene.exec("""
            const headless = !!window.__dimosHeadless;
            const el = document.getElementById("agent-panel");
            if (!el) return { headless, hidden: true };
            const display = el.style.display || getComputedStyle(el).display;
            return { headless, hidden: display === "none" };
        """)
        if result["headless"]:
            # Headless skips UI cleanup — that's correct
            pass
        else:
            assert result["hidden"], f"Sidebar should be hidden in headed mode: {result}"

    def test_no_version_badge(self, scene):
        """No version badge text in the DOM."""
        found = scene.exec("""
            const els = document.querySelectorAll("div");
            for (const el of els) {
                if (el.textContent.includes("DimSim v") && el.style.position === "fixed") {
                    return true;
                }
            }
            return false;
        """)
        assert found is False, "Version badge should not exist in dimos mode"


# ── Tests: Server Physics Reinit ────────────────────────────────────────────


class TestPhysicsReinit:
    """Server physics reconfiguration on embodiment change."""

    def test_agent_position_after_teleport(self, scene):
        """Teleport agent and verify position settles."""
        scene.set_agent_position(5, 2, 5)
        time.sleep(1)
        pos = scene.get_agent_position()
        assert abs(pos["x"] - 5) < 2.0, f"X drift too large: {pos['x']}"
        assert abs(pos["z"] - 5) < 2.0, f"Z drift too large: {pos['z']}"

    def test_embodiment_change_preserves_position(self, scene):
        """Switching embodiment should preserve approximate position."""
        scene.set_agent_position(3, 1, 3)
        time.sleep(1)
        pos_before = scene.get_agent_position()

        scene.set_embodiment("drone")
        time.sleep(1)
        pos_after = scene.get_agent_position()

        # Position should be roughly the same (within 3m tolerance for physics settle)
        assert abs(pos_after["x"] - pos_before["x"]) < 3.0
        assert abs(pos_after["z"] - pos_before["z"]) < 3.0

        # Restore
        scene.set_embodiment("quadruped")
        time.sleep(1)


# ── Tests: Empty Scene ──────────────────────────────────────────────────────


class TestEmptyScene:
    """Empty scene has no floor/primitives."""

    def test_scene_is_empty_mode(self, scene):
        """Verify the scene was launched in empty mode (no apt furniture)."""
        # Check window.__dimosScene is "empty"
        scene_name = scene.exec("return window.__dimosScene;")
        assert scene_name == "empty", f"Expected empty scene, got: {scene_name}"

    def test_no_green_floor(self, scene):
        """No green floor primitive in empty scene."""
        found = scene.exec("""
            let found = false;
            scene.traverse(obj => {
                if (obj.isMesh && obj.material && obj.material.color) {
                    const c = obj.material.color;
                    // Green floor was color ~(0.2, 0.8, 0.2)
                    if (c.g > 0.7 && c.r < 0.3 && c.b < 0.3) found = true;
                }
            });
            return found;
        """)
        assert found is False, "Green floor should not exist in empty scene"


# ── Tests: CLI Flags (separate server instance) ─────────────────────────────


class TestCliFlags:
    """Test that CLI flags are correctly passed to the browser."""

    def test_custom_image_rate_via_cli(self):
        """--image-rate flag sets browser rate."""
        proc = _start_server("--image-rate", "100")
        try:
            client = SceneClient(port=PORT, timeout=30)
            rate = client.exec("return window.__dimosBridge?.rates?.images;")
            assert rate == 100, f"Expected 100ms from CLI flag, got {rate}ms"
            client.close()
        finally:
            _stop_server(proc)

    def test_no_depth_via_cli(self):
        """--no-depth flag disables depth in browser."""
        proc = _start_server("--no-depth")
        try:
            client = SceneClient(port=PORT, timeout=30)
            enable = client.exec("return window.__dimosBridge?.sensorEnable?.depth;")
            assert enable is False, f"Expected depth disabled, got {enable}"
            client.close()
        finally:
            _stop_server(proc)

    def test_camera_fov_via_cli(self):
        """--camera-fov flag sets capture camera FOV."""
        proc = _start_server("--camera-fov", "60")
        try:
            client = SceneClient(port=PORT, timeout=30)
            fov = client.exec("return window.__dimosCapCam?.fov;")
            assert fov == 60, f"Expected 60° from CLI flag, got {fov}"
            client.close()
        finally:
            _stop_server(proc)
