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

"""Rotation helpers, the mount rotation, and the t=0 anchoring for the ghost.

The anchoring tests pass ``mount=np.eye(3)`` so they exercise the transform
itself rather than the fitted mount, which is data and belongs in its own test.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.motion.simulation.vive import mat_to_quat, quat_to_mat

QUATS = [
    (1.0, 0.0, 0.0, 0.0),  # identity
    (0.0, 1.0, 0.0, 0.0),  # 180 about x — the tracker's mounting flip
    (0.7071068, 0.0, 0.0, 0.7071068),  # 90 about z
    (0.5, 0.5, -0.5, 0.5),
    (0.0, 0.0, 0.7071068, 0.7071068),
]


@pytest.mark.parametrize("q", QUATS)
def test_quat_mat_roundtrip(q):
    m = quat_to_mat(np.array(q))
    back = mat_to_quat(m)
    # q and -q are the same rotation
    if np.dot(back, q) < 0:
        back = -back
    np.testing.assert_allclose(back, q, atol=1e-6)


@pytest.mark.parametrize("q", QUATS)
def test_quat_to_mat_is_orthonormal(q):
    m = quat_to_mat(np.array(q))
    np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(m) == pytest.approx(1.0)


def test_quat_to_mat_normalizes_input():
    scaled = quat_to_mat(np.array([2.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(scaled, np.eye(3), atol=1e-12)


def test_quat_to_mat_is_vectorized():
    q = np.array(QUATS)
    stacked = quat_to_mat(q)
    assert stacked.shape == (len(QUATS), 3, 3)
    for i, one in enumerate(QUATS):
        np.testing.assert_allclose(stacked[i], quat_to_mat(np.array(one)), atol=1e-12)


def test_rotation_applies_in_the_expected_direction():
    # 90 deg about z takes +x to +y
    m = quat_to_mat(np.array([0.7071068, 0.0, 0.0, 0.7071068]))
    np.testing.assert_allclose(m @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-6)


def _write(tmp_path, pos, quat, *, walk_at=None, t_host_shift=None, heights_at=()):
    """A minimal mcap carrying vive/pose (and optionally control_log entries)."""
    import json

    Writer = pytest.importorskip("mcap.writer").Writer

    path = tmp_path / "v.mcap"
    with path.open("wb") as f:
        w = Writer(f)
        w.start()
        sid = w.register_schema("go2/VivePose", "jsonschema", b"{}")
        cid = w.register_channel("vive/pose", "json", sid)
        for i, (p, q) in enumerate(zip(pos, quat, strict=True)):
            d = {"p": list(p), "q": list(q)}
            if t_host_shift is not None:
                d["t_host"] = i * 0.01 + t_host_shift
            payload = json.dumps(d).encode()
            w.add_message(cid, log_time=int(i * 1e7), data=payload, publish_time=int(i * 1e7))
        if walk_at is not None or heights_at:
            wid = w.register_channel("control_log", "json", sid)
        if walk_at is not None:
            payload = json.dumps({"action": "walk", "vx": 0.5, "vy": 0.0, "vyaw": 0.0}).encode()
            w.add_message(wid, log_time=int(walk_at * 1e9), data=payload, publish_time=0)
        for at, gh in heights_at:
            payload = json.dumps({"action": "gait_height", "gh": gh, "type": "policy"}).encode()
            w.add_message(wid, log_time=int(at * 1e9), data=payload, publish_time=0)
        w.finish()
    return path


def test_base_track_anchors_first_sample_at_origin(tmp_path):
    from dimos.navigation.motion.simulation.vive import base_track

    pos = [[5.0, 5.0, 1.0], [5.5, 5.0, 1.0], [6.0, 5.0, 1.0]]
    quat = [[1.0, 0, 0, 0]] * 3
    t, p, q = base_track(_write(tmp_path, pos, quat), tracker_offset=np.zeros(3), mount=np.eye(3))

    np.testing.assert_allclose(t, [0.0, 0.01, 0.02], atol=1e-9)
    # the Vive room origin is irrelevant: t=0 lands at 0
    np.testing.assert_allclose(p[0], [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(p[-1], [1.0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(q[0], [1, 0, 0, 0], atol=1e-9)


def test_tracker_offset_is_a_lever_arm_not_a_shift(tmp_path):
    """A pure offset cancels while the body holds still, and bites once it turns."""
    from dimos.navigation.motion.simulation.vive import base_track

    yaw90 = [0.7071068, 0.0, 0.0, 0.7071068]
    path = _write(tmp_path, [[0.0, 0, 2.0], [0.0, 0, 2.0]], [[1.0, 0, 0, 0], yaw90])
    _t, p, _q = base_track(path, tracker_offset=np.array([0.3, 0.0, -0.15]), mount=np.eye(3))

    # stationary tracker, so any motion here is the lever arm swinging
    np.testing.assert_allclose(p[0], [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(p[1], [-0.3, 0.3, 0.0], atol=1e-6)


def test_anchor_pos_offsets_the_whole_track(tmp_path):
    from dimos.navigation.motion.simulation.vive import base_track

    path = _write(tmp_path, [[0.0, 0, 0], [1.0, 0, 0]], [[1.0, 0, 0, 0]] * 2)
    _t, p, _q = base_track(
        path, tracker_offset=np.zeros(3), mount=np.eye(3), anchor_pos=np.array([0, 0, 0.27])
    )
    np.testing.assert_allclose(p[0], [0, 0, 0.27], atol=1e-9)
    np.testing.assert_allclose(p[1], [1.0, 0, 0.27], atol=1e-9)


def test_vive_time_is_zeroed_at_the_first_walk_command(tmp_path):
    """Both streams must share an epoch — zeroing each at its own first message
    made himloco01's vive samples 0.313 s early, which a search then "fitted"
    as 0.317 s of command delay."""
    from dimos.navigation.motion.simulation.vive import read_vive_pose

    pos = [[0.0, 0, 0]] * 5
    quat = [[1.0, 0, 0, 0]] * 5
    t, _p, _q = read_vive_pose(_write(tmp_path, pos, quat, walk_at=0.02))
    np.testing.assert_allclose(t, [-0.02, -0.01, 0.0, 0.01, 0.02], atol=1e-9)


def test_vive_epoch_survives_a_t_host_clock_offset(tmp_path):
    """t_host provides the sample spacing but must not shift the epoch: the
    mean t_host-to-log_time difference rebases it onto the command clock."""
    from dimos.navigation.motion.simulation.vive import read_vive_pose

    pos = [[0.0, 0, 0]] * 5
    quat = [[1.0, 0, 0, 0]] * 5
    t, _p, _q = read_vive_pose(_write(tmp_path, pos, quat, walk_at=0.02, t_host_shift=-5.0))
    np.testing.assert_allclose(t, [-0.02, -0.01, 0.0, 0.01, 0.02], atol=1e-9)


def test_vive_time_without_commands_starts_at_zero(tmp_path):
    from dimos.navigation.motion.simulation.vive import read_vive_pose

    t, _p, _q = read_vive_pose(_write(tmp_path, [[0.0, 0, 0]] * 3, [[1.0, 0, 0, 0]] * 3))
    np.testing.assert_allclose(t, [0.0, 0.01, 0.02], atol=1e-9)


def test_sensor_z_keeps_the_raw_tracker_height(tmp_path):
    """With sensor_z the z column must bypass both the lever arm and the anchor."""
    from dimos.navigation.motion.simulation.vive import base_track

    pos = [[5.0, 5.0, 1.4], [5.5, 5.0, 1.5]]
    quat = [[1.0, 0, 0, 0]] * 2
    _t, p, _q = base_track(
        _write(tmp_path, pos, quat),
        tracker_offset=np.array([0.0, 0.0, -0.2]),
        mount=np.eye(3),
        anchor_pos=np.array([0.0, 0.0, 0.27]),
        sensor_z=True,
    )
    np.testing.assert_allclose(p[:, 2], [1.4, 1.5], atol=1e-9)
    np.testing.assert_allclose(p[:, 0], [0.0, 0.5], atol=1e-9)  # xy still anchored


def _pack_lowcmd(q_sdk):
    """CDR bytes mirroring walk._parse_lowcmd_q's layout, 12 motors."""
    import struct

    b = bytearray(26)
    for q in q_sdk:
        b += b"\x01"
        b += b"\x00" * (-len(b) % 4)
        b += struct.pack("<5f", q, 0.0, 0.0, 40.0, 1.0)
        b += b"\x00" * 12
    return bytes(b)


def test_read_policy_lowcmd_remaps_and_shares_the_epoch(tmp_path):
    """SDK motor order in, training order out; t=0 at the first walk command."""
    import json

    Writer = pytest.importorskip("mcap.writer").Writer

    from dimos.navigation.motion.simulation.walk import read_policy_lowcmd

    q_sdk = [10.0, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42]  # FR FL RR RL
    path = tmp_path / "cmd.mcap"
    with path.open("wb") as f:
        w = Writer(f)
        w.start()
        sid = w.register_schema("s", "jsonschema", b"{}")
        low = w.register_channel("policy/lowcmd", "cdr", sid)
        ctl = w.register_channel("control_log", "json", sid)
        for i in range(3):
            w.add_message(low, log_time=int(i * 1e7), data=_pack_lowcmd(q_sdk), publish_time=0)
        walk_cmd = json.dumps({"action": "walk", "vx": 0.1, "vy": 0.0, "vyaw": 0.0}).encode()
        w.add_message(ctl, log_time=int(2e7), data=walk_cmd, publish_time=0)
        w.finish()

    t, q = read_policy_lowcmd(path)
    np.testing.assert_allclose(t, [-0.02, -0.01, 0.0], atol=1e-9)
    # training order FL,FR,RL,RR
    np.testing.assert_allclose(q[0], [20, 21, 22, 10, 11, 12, 40, 41, 42, 30, 31, 32])


def test_read_gait_height_shares_the_epoch(tmp_path):
    """Height commands must land on the same clock as everything else: t=0 at
    the first walk command, not at the first slider touch."""
    from dimos.navigation.motion.simulation.walk import read_gait_height

    path = _write(
        tmp_path,
        [[0.0, 0, 0]] * 2,
        [[1.0, 0, 0, 0]] * 2,
        walk_at=0.02,
        heights_at=[(0.01, 0.2), (0.05, 0.33)],
    )
    t, h = read_gait_height(path)
    np.testing.assert_allclose(t, [-0.01, 0.03], atol=1e-9)
    np.testing.assert_allclose(h, [0.2, 0.33])


def test_read_gait_height_is_empty_without_height_commands(tmp_path):
    from dimos.navigation.motion.simulation.walk import read_gait_height

    t, h = read_gait_height(_write(tmp_path, [[0.0, 0, 0]] * 2, [[1.0, 0, 0, 0]] * 2, walk_at=0.0))
    assert len(t) == 0 and len(h) == 0


def test_mount_rotation_is_a_proper_rotation():
    from dimos.navigation.motion.simulation.vive import mount_rotation

    for yaw in (0.0, 45.0, 94.0, 270.0):
        m = mount_rotation(yaw)
        np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(m) == pytest.approx(1.0)


def test_mount_maps_robot_forward_onto_the_tracker_axis():
    """At the fitted 94 deg, robot +x lands ~along tracker +y — the "mirror"."""
    from dimos.navigation.motion.simulation.vive import mount_rotation

    fwd_in_tracker = mount_rotation(94.0) @ np.array([1.0, 0.0, 0.0])
    assert fwd_in_tracker[1] > 0.99
    assert abs(fwd_in_tracker[0]) < 0.08


def test_mount_flip_puts_robot_up_against_tracker_down():
    from dimos.navigation.motion.simulation.vive import mount_rotation

    up_in_tracker = mount_rotation(94.0, flip=True) @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(up_in_tracker, [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(
        mount_rotation(94.0, flip=False) @ np.array([0.0, 0.0, 1.0]), [0, 0, 1], atol=1e-12
    )
