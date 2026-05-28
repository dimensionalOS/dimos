"""Container-level tests for the PawTrack subject-tracking skill glue.

Exercise the DimOS ``PawTrackSkillContainer`` orchestration -- monitor state
reset, malformed input, status publishing, and the ground-pose raycast -- with
fakes for the EdgeTAM tracker and the output streams, so no GPU, robot, or
network is needed.
"""

import json

import numpy as np
import pytest

from pawtrack.ground_raycast import CameraIntrinsics
from pawtrack.skill_container import PawTrackSkillContainer
from pawtrack.track_state import TrackMetrics


class _Recorder:
    """Stand-in for an ``Out`` stream that records published messages."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeImage:
    """Minimal ``Image`` stand-in: size for bbox math, a frame for overlays."""

    width = 640
    height = 480
    ts = 0.0

    def to_opencv(self):
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)


class _FakeTracker:
    """EdgeTAM stand-in; ``init_count`` controls init success vs failure."""

    def __init__(self, init_count=0):
        self._init_count = init_count

    def init_track(self, **_kwargs):
        return list(range(self._init_count))

    def process_image(self, _image):
        return []  # always a miss, so the monitor loop just coasts

    def stop(self):
        pass


class _FakeTransform:
    """Transform stand-in exposing the matrix API used by the raycast."""

    def __init__(self, matrix):
        self._matrix = matrix

    def to_matrix(self):
        return self._matrix


class _FakeTf:
    """Frame lookup table for pose-publishing tests."""

    def __init__(self, transforms):
        self._transforms = transforms

    def get(self, frame_id, _child_frame_id, **_kwargs):
        return self._transforms.get(frame_id)


def _look_down_transform(x=0.0, y=0.0, z=1.0):
    transform = np.array(
        [
            [1.0, 0.0, 0.0, x],
            [0.0, -1.0, 0.0, y],
            [0.0, 0.0, -1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    return _FakeTransform(transform)


# A bbox whose bottom-center -- the ground-contact pixel -- is (320, 240).
_CONTACT_BBOX = (300.0, 200.0, 340.0, 240.0)


def _container(init_count=0):
    container = PawTrackSkillContainer()
    container.subject_status = _Recorder()
    container.debug_image = _Recorder()
    container.subject_world_pose = _Recorder()
    container.subject_map_pose = _Recorder()
    container._latest_image = _FakeImage()
    container._tracker = _FakeTracker(init_count)
    return container


def _statuses(container):
    return [json.loads(msg)["status"] for msg in container.subject_status.msgs]


def _metrics():
    return TrackMetrics(
        bbox=(10.0, 10.0, 30.0, 30.0), center_px=(20.0, 20.0),
        width_px=20.0, height_px=20.0, area_px=400.0, area_ratio=0.001,
        image_error_x=0.0, image_error_y=0.0, confidence=0.9,
        mask_area_px=320.0, mask_area_ratio=0.001,
    )


# -- Perception --

def test_track_subject_rejects_malformed_initial_bbox():
    container = _container()
    result = container.track_subject("a person", initial_bbox=[1, 2])
    assert "initial_bbox" in result
    assert container.subject_status.msgs == []  # nothing published
    assert container._snapshot.status == "idle"  # state left untouched


def test_track_subject_resets_prior_state_and_publishes_acquiring():
    container = _container(init_count=0)  # tracker init fails -> no loop thread
    # A prior track left a baseline behind.
    container._state.observe("a prior subject", _metrics(), 1.0)
    assert container._state._last_tracked is not None

    result = container.track_subject(
        "a person sitting on a chair", initial_bbox=[10.0, 10.0, 60.0, 60.0])

    assert "Could not initialize" in result
    assert _statuses(container) == ["acquiring", "lost"]
    first = json.loads(container.subject_status.msgs[0])
    assert first["description"] == "a person sitting on a chair"
    # The old target's baseline was cleared, so it cannot carry forward.
    assert container._state._last_tracked is None


def test_track_subject_defaults_to_a_seated_person():
    container = _container(init_count=0)
    container.track_subject(initial_bbox=[10.0, 10.0, 60.0, 60.0])
    first = json.loads(container.subject_status.msgs[0])
    assert first["description"] == "a person sitting on a chair"


def test_status_json_is_well_formed():
    container = _container(init_count=0)
    container.track_subject(
        "a person sitting on a chair", initial_bbox=[10.0, 10.0, 60.0, 60.0])
    assert container.subject_status.msgs
    for msg in container.subject_status.msgs:
        data = json.loads(msg)
        assert "seen_at" not in data
        assert "last_seen_age_s" in data
        for key in ("status", "description", "message"):
            assert key in data


def test_start_then_stop_publishes_stopped():
    container = _container(init_count=1)  # tracker init succeeds -> loop starts
    result = container.track_subject(
        "a person sitting on a chair", initial_bbox=[10.0, 10.0, 60.0, 60.0])
    assert result.startswith("Started tracking")

    stopped = container.stop_tracking()

    assert stopped.startswith("Stopped")
    assert container._snapshot.status == "stopped"
    statuses = _statuses(container)
    assert "acquiring" in statuses and "stopped" in statuses
    assert container._thread is None or not container._thread.is_alive()


def test_ground_pose_publishes_world_and_optional_map():
    container = _container()
    container._intrinsics = CameraIntrinsics(
        fx=100.0, fy=100.0, cx=320.0, cy=240.0)
    container._tf = _FakeTf({
        "world": _look_down_transform(x=1.0),
        "map": _look_down_transform(x=10.0),
    })

    container._publish_ground_poses(_FakeImage(), _CONTACT_BBOX)

    assert len(container.subject_world_pose.msgs) == 1
    assert len(container.subject_map_pose.msgs) == 1
    assert container.subject_world_pose.msgs[0].frame_id == "world"
    assert container.subject_map_pose.msgs[0].frame_id == "map"
    # The contact pixel rests on the floor, so z is the floor height (0).
    assert tuple(
        container.subject_world_pose.msgs[0].position) == pytest.approx(
        (1.0, 0.0, 0.0))
    assert tuple(
        container.subject_map_pose.msgs[0].position) == pytest.approx(
        (10.0, 0.0, 0.0))


def test_ground_pose_keeps_world_when_map_tf_missing():
    container = _container()
    container._intrinsics = CameraIntrinsics(
        fx=100.0, fy=100.0, cx=320.0, cy=240.0)
    container._tf = _FakeTf({"world": _look_down_transform()})

    container._publish_ground_poses(_FakeImage(), _CONTACT_BBOX)

    assert len(container.subject_world_pose.msgs) == 1
    assert container.subject_world_pose.msgs[0].frame_id == "world"
    assert container.subject_map_pose.msgs == []


def test_absent_map_tf_lookup_is_throttled_not_per_frame():
    # tf.get logs on every miss, so a missing map frame must not be probed each
    # tracked frame. World is present (probed every frame); map is absent
    # (probed once, then throttled).
    container = _container()
    container._intrinsics = CameraIntrinsics(
        fx=100.0, fy=100.0, cx=320.0, cy=240.0)
    calls = {}

    class _CountingTf:
        def get(self, frame_id, _child, **_kwargs):
            calls[frame_id] = calls.get(frame_id, 0) + 1
            return _look_down_transform() if frame_id == "world" else None

    container._tf = _CountingTf()
    for _ in range(5):
        container._publish_ground_poses(_FakeImage(), _CONTACT_BBOX)

    assert calls["world"] == 5  # present -> probed every frame
    assert calls["map"] == 1  # absent -> one probe, then throttled
    assert container.subject_map_pose.msgs == []


def test_map_lookup_uses_looser_tolerance_than_world():
    # The map<-world TF is republished slowly, so the map lookup must tolerate
    # a staler transform than the live odom chain, or it pauses mid-interval.
    container = _container()
    container._intrinsics = CameraIntrinsics(
        fx=100.0, fy=100.0, cx=320.0, cy=240.0)
    seen = {}

    class _CaptureTf:
        def get(self, frame_id, _child, *, time_point=None, time_tolerance=None):
            seen[frame_id] = time_tolerance
            return _look_down_transform()

    container._tf = _CaptureTf()
    container._publish_ground_poses(_FakeImage(), _CONTACT_BBOX)

    assert seen["map"] > seen["world"]
    assert seen["map"] >= 2.0  # must exceed the relocalization publish interval
