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

from collections.abc import Iterator
import io
import json
import time

from dimos_lcm.vision_msgs import (
    BoundingBox3D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.agents.typesafe.world_state import Memory
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.robot.raw_robot_bridge import (
    Deadman,
    RawTopics,
    drive_words,
    jpeg_bytes,
    odom_json,
    world_state_json,
    xyz_f32,
)

ENDPOINT = "tcp/127.0.0.1:17448"


def wait_for(condition, timeout_s: float = 5.0) -> None:  # type: ignore[no-untyped-def]
    deadline = time.monotonic() + timeout_s
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


@pytest.fixture
def topics() -> Iterator[tuple[RawTopics, RawTopics]]:
    robot = RawTopics(ENDPOINT, listen=True)
    client = RawTopics(ENDPOINT, listen=False)
    try:
        yield robot, client
    finally:
        client.close()
        robot.close()


def test_client_receives_sensor_topics_and_robot_receives_commands(
    topics: tuple[RawTopics, RawTopics],
) -> None:
    robot, client = topics
    got: list[tuple[str, bytes, float | None]] = []
    client.subscribe("odom/json", lambda b, ts: got.append(("odom", b, ts)))
    client.subscribe("camera/jpeg", lambda b, ts: got.append(("camera", b, ts)))
    commands: list[bytes] = []
    robot.subscribe("cmd_vel/json", lambda b, _ts: commands.append(b))
    time.sleep(0.5)  # subscriptions propagate over the link

    pose = PoseStamped(position=(1.0, 2.0, 0.5), orientation=(0.0, 0.0, 0.0, 1.0), ts=3.5)
    robot.put("odom/json", odom_json(pose))
    robot.put("camera/jpeg", b"\xff\xd8jpeg", ts=4.25)
    client.put("cmd_vel/json", json.dumps({"vx": 0.3, "vy": 0.0, "wz": 0.1, "t": 1.0}))
    wait_for(lambda: len(got) == 2 and len(commands) == 1)

    odom = json.loads(next(b for k, b, _ in got if k == "odom"))
    assert (odom["t"], odom["x"], odom["y"], odom["z"], odom["qw"]) == (3.5, 1.0, 2.0, 0.5, 1.0)
    assert next((b, ts) for k, b, ts in got if k == "camera") == (b"\xff\xd8jpeg", 4.25)
    assert json.loads(commands[0])["vx"] == 0.3


def test_encoders_are_plain_formats() -> None:
    frame = np.zeros((6, 8, 3), dtype=np.uint8)
    frame[:, :, 0] = 200
    decoded = PILImage.open(io.BytesIO(jpeg_bytes(Image.from_numpy(frame, ts=1.0))))
    assert decoded.format == "JPEG" and decoded.size == (8, 6)

    points = np.arange(12, dtype=np.float32).reshape(4, 3)
    raw = xyz_f32(PointCloud2.from_numpy(points, timestamp=2.0))
    assert np.frombuffer(raw, dtype="<f4").reshape(-1, 3).tolist() == points.tolist()

    fields = json.loads(
        odom_json(PoseStamped(position=(0, 0, 0), orientation=(0, 0, 0, 1), ts=9.0))
    )
    assert list(fields) == ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]


def test_deadman_holds_then_stops_and_caps_the_hold() -> None:
    deadman = Deadman(max_s=0.2)
    deadman.set(json.dumps({"vx": 0.5, "wz": -0.2, "t": 0.05}))
    assert deadman.current() == (0.5, 0.0, -0.2)
    time.sleep(0.08)
    assert deadman.current() == (0.0, 0.0, 0.0)

    deadman.set(json.dumps({"vx": 1.0, "t": 60}))  # capped to max_s
    time.sleep(0.25)
    assert deadman.current() == (0.0, 0.0, 0.0)

    with pytest.raises(ValueError):
        deadman.set(b"not json")


def test_deadman_rejects_non_finite_and_clamps_speed() -> None:
    deadman = Deadman(max_s=2.0, max_linear=1.0, max_angular=1.5)
    for bad in (
        '{"vx": NaN, "t": 1}',
        '{"vx": Infinity, "t": 1}',
        '{"wz": -Infinity, "t": 1}',
        '{"vx": 1, "t": NaN}',
    ):
        with pytest.raises(ValueError):
            deadman.set(bad)
    assert deadman.current() == (0.0, 0.0, 0.0)  # nothing armed by the bad packets
    deadman.set(json.dumps({"vx": 1e308, "vy": -7.0, "wz": 40.0, "t": 1}))
    assert deadman.current() == (1.0, -1.0, 1.5)


@pytest.mark.parametrize(
    "kwargs", [{"max_linear": -1.0}, {"max_angular": 0.0}, {"max_s": float("inf")}]
)
def test_deadman_rejects_non_positive_limits(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match="finite positive limit"):
        Deadman(**kwargs)


def _couch_at(x: float, y: float) -> Detection3DArray:
    d = Detection3D()
    d.header = Header(1.0, "world")
    d.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id="couch", score=0.9))]
    d.results_length = 1
    d.bbox = BoundingBox3D(center=Pose(position=(x, y, 0.3)), size=Vector3(1.5, 0.8, 0.6))
    return Detection3DArray(detections_length=1, header=Header(1.0, "world"), detections=[d])


def test_world_state_is_the_typesafe_document() -> None:
    pose = PoseStamped(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0), ts=1.0)
    state = json.loads(
        world_state_json(
            "go to the couch at (3.00, 0.00)",
            pose,
            detections_3d=_couch_at(3.0, 0.0),
            lidar=None,
            cmd=(0.3, 0.0, 0.0),
            memory=Memory(),
            now=1.0,
            lidar_band=(0.1, 0.8, 5.0),
        )
    )
    assert set(state) >= {"task", "goal", "robot", "objects", "way_to_target", "free_space"}
    assert state["objects"][0]["label"] == "couch" and state["objects"][0]["target"] is True
    assert "position" not in state["objects"][0] and "(3.00" not in state["goal"]
    assert state["robot"]["motion"] == "driving"
    assert state["robot"]["last_drive"] == {"x": "forward", "y": "none", "yaw": "none"}
    assert drive_words(0.0, -0.2, 0.5) == {"x": "none", "y": "right", "yaw": "turn_left"}


def test_world_state_tick_reports_a_builder_fault_instead_of_dying() -> None:
    from dimos.robot.raw_robot_bridge import world_state_tick

    stats = {"ticks": 0, "errors": 0, "last_error": ""}

    def bad() -> str:
        raise IndexError("list index out of range")

    payload = json.loads(world_state_tick(bad, stats))
    assert payload["error"] == "world state unavailable" and "IndexError" in payload["detail"]
    assert world_state_tick(lambda: '{"ok": 1}', stats) == '{"ok": 1}'
    assert (stats["ticks"], stats["errors"]) == (1, 1)


def test_dry_run_builds_the_first_tick_from_scene_objects() -> None:
    from dimos.robot.raw_robot_bridge import dry_run_world_state

    objects = [
        (
            "wall",
            (0.0, 3.0, 1.0),
            (10.0, 0.01, 2.5),
        ),  # a 1 cm wall used to crash the doorway search
        ("chair", (4.0, 1.0, 0.4), (0.6, 0.6, 0.8)),
    ]
    doc = json.loads(
        dry_run_world_state("go to the chair at (4.00, 1.00)", (0.0, 0.0, 0.0), 0.0, objects)
    )
    assert doc["objects"][0]["label"] == "chair" and doc["way_to_target"]["state"] in (
        "clear",
        "blocked",
    )
