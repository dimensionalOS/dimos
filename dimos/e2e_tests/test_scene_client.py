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

"""End-to-end tests for SceneClient against a running DimSim instance.

Requires a running dimsim server (headless or headed):

    # Headless (CI):
    dimsim dev --scene empty --headless

    # Headed (local dev):
    dimsim dev --scene empty

Run:
    pytest dimos/e2e_tests/test_scene_client.py -v -s -m slow
"""

import os
import signal
import socket
import subprocess
import time

import pytest

from dimos.robot.sim.scene_client import SceneClient, SceneExecError

PORT = 8090
SOLDIER_URL = "https://threejs.org/examples/models/gltf/Soldier.glb"


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


@pytest.fixture(scope="module")
def dimsim_server():
    """Start a headless dimsim server for the test module."""
    _force_kill_port(PORT)
    time.sleep(1)

    proc = subprocess.Popen(
        ["dimsim", "dev", "--scene", "empty", "--headless", "--render", "cpu"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    assert _wait_for_port(PORT, timeout=120), "dimsim failed to start"
    # Wait for browser page to load
    time.sleep(10)

    yield proc

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture
def scene(dimsim_server):
    """Create a SceneClient connected to the running dimsim."""
    client = SceneClient(port=PORT, timeout=30)
    yield client
    client.close()


@pytest.mark.skipif_in_ci
@pytest.mark.slow
class TestSceneClient:
    """SceneClient SDK tests against a live dimsim instance."""

    def test_exec_basic(self, scene):
        """Raw exec returns a value."""
        result = scene.exec("return 1 + 2;")
        assert result == 3

    def test_exec_string(self, scene):
        result = scene.exec("return 'hello world';")
        assert result == "hello world"

    def test_exec_object(self, scene):
        result = scene.exec("return { a: 1, b: 'two' };")
        assert result["a"] == 1
        assert result["b"] == "two"

    def test_exec_error(self, scene):
        """Bad JS raises SceneExecError."""
        with pytest.raises(SceneExecError):
            scene.exec("throw new Error('test error');")

    def test_exec_await(self, scene):
        """Async code works (top-level await)."""
        result = scene.exec("""
            const p = new Promise(r => setTimeout(() => r(42), 100));
            return await p;
        """)
        assert result == 42

    def test_get_scene_info(self, scene):
        """get_scene_info returns object list."""
        info = scene.get_scene_info()
        assert "objectCount" in info
        assert "objects" in info
        assert isinstance(info["objects"], list)

    def test_get_agent_position(self, scene):
        """Agent position is a dict with x, y, z."""
        pos = scene.get_agent_position()
        assert "x" in pos and "y" in pos and "z" in pos
        assert isinstance(pos["x"], (int, float))

    def test_set_agent_position(self, scene):
        """Teleport agent and verify position."""
        scene.set_agent_position(5, 1, 3)
        time.sleep(0.5)  # physics settle
        pos = scene.get_agent_position()
        assert abs(pos["x"] - 5) < 1.0
        assert abs(pos["z"] - 3) < 1.0

    def test_set_background(self, scene):
        """set_background doesn't error."""
        scene.set_background(0xFF0000)

    def test_set_fog(self, scene):
        """set_fog creates fog on the scene."""
        scene.set_fog(color=0xCCCCCC, near=5, far=30)
        result = scene.exec(
            "return scene.fog ? { near: scene.fog.near, far: scene.fog.far } : null;"
        )
        assert result is not None
        assert result["near"] == 5
        assert result["far"] == 30

    def test_exec_add_mesh(self, scene):
        """Add a mesh via raw exec."""
        result = scene.exec("""
            const mesh = new THREE.Mesh(
                new THREE.BoxGeometry(1, 1, 1),
                new THREE.MeshStandardMaterial({ color: 0xff0000 })
            );
            mesh.name = "test-box-exec";
            scene.add(mesh);
            return { name: mesh.name, uuid: mesh.uuid };
        """)
        assert result["name"] == "test-box-exec"

    def test_remove_object(self, scene):
        """Add then remove an object by name."""
        scene.exec("""
            const m = new THREE.Mesh(
                new THREE.BoxGeometry(1, 1, 1),
                new THREE.MeshStandardMaterial({ color: 0x00ff00 })
            );
            m.name = "removable-box";
            scene.add(m);
        """)
        removed = scene.remove_object("removable-box")
        assert removed is True

        # Verify it's gone
        result = scene.exec("""
            return scene.getObjectByName("removable-box") ? true : false;
        """)
        assert result is False

    def test_remove_object_not_found(self, scene):
        """Removing nonexistent object returns False."""
        removed = scene.remove_object("does-not-exist-12345")
        assert removed is False

    def test_add_collider_box(self, scene):
        """Add a box collider to a mesh."""
        scene.exec("""
            const m = new THREE.Mesh(
                new THREE.BoxGeometry(2, 2, 2),
                new THREE.MeshStandardMaterial({ color: 0x0000ff })
            );
            m.name = "collider-test-box";
            m.position.set(10, 1, 10);
            scene.add(m);
        """)
        result = scene.add_collider("collider-test-box", shape="box")
        assert result["shape"] == "box"
        assert "uuid" in result

        # Cleanup
        scene.remove_object("collider-test-box")

    def test_load_map(self, scene):
        """Load a GLTF map with collider."""
        result = scene.load_map(
            url="/proxy?url=https://threejs.org/examples/models/gltf/collision-world.glb",
            name="test-map",
            position=(0, 0, 0),
            collider="trimesh",
        )
        assert result["name"] == "test-map"
        assert result["collider"] is not None

        # Cleanup
        scene.remove_object("test-map")

    def test_add_npc(self, scene):
        """Add an animated NPC."""
        npc = scene.add_npc(
            url=SOLDIER_URL,
            name="test-soldier",
            position=(2, 0, 2),
            animation="Walk",
        )
        assert npc["name"] == "test-soldier"
        assert "Walk" in npc["activeAnimation"]
        assert len(npc["animations"]) > 0

    def test_remove_npc(self, scene):
        """Remove an NPC by name."""
        scene.add_npc(
            url=SOLDIER_URL,
            name="npc-to-remove",
            position=(8, 0, 8),
            animation=0,
        )
        removed = scene.remove_npc("npc-to-remove")
        assert removed is True

    def test_clear_scene(self, scene):
        """clear_scene removes user objects but keeps agent/camera/lights."""
        # Add some objects
        scene.exec("""
            for (let i = 0; i < 3; i++) {
                const m = new THREE.Mesh(
                    new THREE.BoxGeometry(1,1,1),
                    new THREE.MeshStandardMaterial()
                );
                m.name = `clear-test-${i}`;
                scene.add(m);
            }
        """)

        before = scene.get_scene_info()
        removed = scene.clear_scene()
        after = scene.get_scene_info()

        assert removed > 0
        assert after["objectCount"] <= before["objectCount"]

    def test_screenshot(self, scene):
        """Screenshot returns a data URL."""
        data_url = scene.screenshot()
        assert data_url.startswith("data:image/png;base64,")
        assert len(data_url) > 100  # non-trivial image

    def test_multiple_execs_sequential(self, scene):
        """Multiple sequential execs work correctly (no ID confusion)."""
        results = []
        for i in range(5):
            r = scene.exec(f"return {i} * {i};")
            results.append(r)
        assert results == [0, 1, 4, 9, 16]

    def test_context_manager(self, dimsim_server):
        """SceneClient works as context manager."""
        with SceneClient(port=PORT) as s:
            result = s.exec("return 'ctx';")
            assert result == "ctx"
        # After exit, should be closed
        assert s._closed is True
