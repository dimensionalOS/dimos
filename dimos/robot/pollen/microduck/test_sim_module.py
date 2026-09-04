# Copyright 2025-2026 Dimensional Inc.
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

"""``MicroduckSimModule``: policy_request parsing, policy_state publishing,
chase-camera plumbing, ball spawning and model composition. Tests that need
MuJoCo + the asset cache are marked ``mujoco`` (``pytest -m mujoco``)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.robot.pollen.microduck import assets_fetch
from dimos.robot.pollen.microduck.places import BALL_BODY, BALL_RADIUS, FOUR_ROOM_XML
from dimos.robot.pollen.microduck.sim_module import (
    CHASE_CAMERA_NAME,
    LIDAR_CAMERA_SPECS,
    PHYSICS_TIMESTEP,
    POV_CAMERA_NAME,
    MicroduckSimModule,
    MicroduckSimModuleConfig,
    _ball_spawn_xy,
    _camera_quat_wxyz,
    _shape_twist,
    _state_key,
)
from dimos.simulation.engines import mujoco_sim_module as msm
from dimos.simulation.engines.mujoco_engine import CameraFrame
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


def _cache_or_skip() -> Path:
    robot_xml = assets_fetch.variant_mjcf_path("default")
    if not robot_xml.exists():
        pytest.skip(f"Microduck asset cache not present ({assets_fetch.assets_root()})")
    return robot_xml


class _FakeScheduler:
    """Records ``request`` calls; stands in for PolicyScheduler in parsing tests."""

    def __init__(self, accept: bool = True) -> None:
        self.requests: list[tuple[str | None, str]] = []
        self.accept = accept
        self.suspend_fall_detector = False
        self.falls: list[bool] = []

    def request(self, policy: str | None, action: str) -> tuple[bool, str]:
        self.requests.append((policy, action))
        return (self.accept, "" if self.accept else "nope")

    def notify_fall(self, fallen: bool) -> None:
        self.falls.append(fallen)


class _FakeBank:
    def __init__(self, gravity_z: float = -1.0, root_qpos_adr: int = 0) -> None:
        self.gravity_z = gravity_z
        self.root_qpos_adr = root_qpos_adr

    def projected_gravity(self, data: Any) -> np.ndarray:
        return np.array([0.0, 0.0, self.gravity_z])

    def root_yaw(self, data: Any) -> float:
        w, x, y, z = data.qpos[self.root_qpos_adr + 3 : self.root_qpos_adr + 7]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _frame(ts: float, shape: tuple[int, int] = (360, 640)) -> CameraFrame:
    return CameraFrame(
        rgb=np.full((*shape, 3), 120, dtype=np.uint8),
        depth=np.ones(shape, dtype=np.float32),
        cam_pos=np.zeros(3),
        cam_mat=np.eye(3),
        fovy=60.0,
        timestamp=ts,
    )


# ---------------------------------------------------------------- pure helpers


def test_config_defaults_match_design() -> None:
    cfg = MicroduckSimModuleConfig()
    assert cfg.dof == 14
    assert cfg.camera_name == "head_camera" and cfg.base_frame_id == "trunk_base"
    assert cfg.variant == "default" and cfg.policy_dir is None
    assert cfg.chase_cam is True
    assert cfg.chase_cam_size == (640, 360) and cfg.chase_cam_fps == 12.0
    assert cfg.chase_cam_offset == (-0.8, 0.0, 0.45)
    assert cfg.chase_cam_pitch_deg == -20.0 and cfg.chase_cam_fovy == 60.0
    assert cfg.ball_body == BALL_BODY
    assert cfg.state_hz == 5.0
    assert cfg.cast_shadows is False
    assert cfg.extra_cameras == {}
    assert (cfg.cmd_gain_linear, cfg.cmd_gain_angular) == (2.4, 2.6)
    # The gait's WZ_RANGE maximum: below it pure turns barely rotate.
    assert cfg.min_effective_wz == 1.5 and cfg.cmd_timeout == 1.0
    assert MicroduckSimModuleConfig(chase_cam=False, ball_body="").ball_body == ""


@pytest.mark.parametrize(
    ("twist", "expected"),
    [
        # Gains only; a forward request stays a forward request.
        ((0.125, 0.0, 0.0), (0.3, 0.0, 0.0)),
        ((0.0, -0.1, 0.0), (0.0, -0.24, 0.0)),
        # The planner's rotate-in-place twist (0.275 * 2.6 = 0.715) is
        # bumped to the effective minimum, keeping its sign.
        ((0.0, 0.0, 0.275), (0.0, 0.0, 1.5)),
        ((0.0, 0.0, -0.275), (0.0, 0.0, -1.5)),
        # ... also while walking (the bump is on |wz| alone).
        ((0.125, 0.0, -0.275), (0.3, 0.0, -1.5)),
        # Requests already past the minimum are left alone (the gait clips).
        ((0.0, 0.0, 0.6), (0.0, 0.0, 1.56)),
        # Noise-level yaw is not a turn request.
        ((0.0, 0.0, 0.01), (0.0, 0.0, 0.026)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ],
)
def test_shape_twist_applies_gains_and_yaw_deadband_bump(
    twist: tuple[float, float, float], expected: tuple[float, float, float]
) -> None:
    got = _shape_twist(MicroduckSimModuleConfig(), *twist)
    assert got == pytest.approx(expected)


def test_shape_twist_honours_configured_gains_and_minimum() -> None:
    cfg = MicroduckSimModuleConfig(cmd_gain_linear=1.0, cmd_gain_angular=1.0, min_effective_wz=0.8)
    assert _shape_twist(cfg, 0.2, 0.0, 0.3) == pytest.approx((0.2, 0.0, 0.8))
    assert _shape_twist(cfg, 0.0, 0.0, -0.9) == pytest.approx((0.0, 0.0, -0.9))


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (0.0, (1.09, 2.042)),
        (math.pi / 2, (1.0 - 0.042, 2.0 + 0.09)),
        (math.pi, (1.0 - 0.09, 2.0 - 0.042)),
        (-math.pi / 2, (1.0 + 0.042, 2.0 - 0.09)),
    ],
)
def test_ball_spawn_xy_rotates_offset_into_trunk_yaw_frame(
    yaw: float, expected: tuple[float, float]
) -> None:
    x, y = _ball_spawn_xy(1.0, 2.0, yaw, 0.09, 0.042)
    assert (x, y) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize(
    ("yaw", "pitch_deg"),
    [(0.0, 0.0), (math.radians(120.0), 0.0), (math.radians(-120.0), 0.0), (0.0, -20.0)],
)
def test_camera_quat_points_camera_minus_z_along_yaw_pitch(yaw: float, pitch_deg: float) -> None:
    from scipy.spatial.transform import Rotation as R

    w, x, y, z = _camera_quat_wxyz(yaw, math.radians(pitch_deg))
    rot = R.from_quat([x, y, z, w])
    forward = rot.apply([0.0, 0.0, -1.0])  # MuJoCo cameras look along -z
    up = rot.apply([0.0, 1.0, 0.0])
    pitch = math.radians(pitch_deg)
    assert forward == pytest.approx(
        [math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)],
        abs=1e-9,
    )
    assert up[2] > 0.0  # image "up" is world up, not mirrored


def test_state_key_drops_timestamp_only() -> None:
    snap = {"active": "walk", "locked": False, "t": 12.5, "oneshot": None}
    assert _state_key(snap) == {"active": "walk", "locked": False, "oneshot": None}
    assert "t" in snap  # not mutated


# ------------------------------------------------------------ policy_state out


def test_policy_state_publishes_on_change_or_when_period_elapsed() -> None:
    module = MicroduckSimModule(state_hz=5.0)
    try:
        published: list[str] = []
        module.policy_state.subscribe(published.append)
        walk = {"active": "walk", "locked": False, "t": 1.0}

        assert module._publish_policy_state(walk, now=100.0) is True  # first ever
        assert module._publish_policy_state({**walk, "t": 1.05}, now=100.05) is False
        assert module._publish_policy_state({**walk, "t": 1.1}, now=100.1) is False
        kick = {"active": "kick_left", "locked": True, "t": 1.15}
        assert module._publish_policy_state(kick, now=100.15) is True  # changed
        assert module._publish_policy_state({**kick, "t": 1.3}, now=100.3) is False
        assert module._publish_policy_state({**kick, "t": 1.36}, now=100.36) is True  # 1/5 s due
        assert module._publish_policy_state({**kick, "t": 1.4}, now=100.4) is False

        assert [json.loads(raw)["active"] for raw in published] == [
            "walk",
            "kick_left",
            "kick_left",
        ]
        assert json.loads(published[-1])["t"] == 1.36
        assert published[0] == '{"active":"walk","locked":false,"t":1.0}'  # compact JSON
    finally:
        module.stop()


# ------------------------------------------------------------ policy_request in


def test_on_policy_request_forwards_valid_requests_to_scheduler() -> None:
    module = MicroduckSimModule()
    try:
        scheduler = _FakeScheduler()
        module._scheduler = scheduler  # type: ignore[assignment]
        module._on_policy_request(json.dumps({"action": "start", "policy": "kick_left", "t": 1.0}))
        module._on_policy_request(json.dumps({"action": "toggle", "policy": "sitstand"}))
        module._on_policy_request(json.dumps({"action": "stop"}))  # bare stop: policy absent
        module._on_policy_request(json.dumps({"action": "stop", "policy": None}))
        assert scheduler.requests == [
            ("kick_left", "start"),
            ("sitstand", "toggle"),
            (None, "stop"),
            (None, "stop"),
        ]
    finally:
        module.stop()


@pytest.mark.parametrize(
    "raw",
    [
        "{not json",
        "",
        json.dumps([1, 2, 3]),
        json.dumps("start"),
        json.dumps({"policy": "walk"}),  # no action
        json.dumps({"action": 7, "policy": "walk"}),
        json.dumps({"action": "start", "policy": ["walk"]}),
    ],
)
def test_on_policy_request_ignores_malformed_payloads(raw: str) -> None:
    module = MicroduckSimModule()
    try:
        scheduler = _FakeScheduler()
        module._scheduler = scheduler  # type: ignore[assignment]
        module._on_policy_request(raw)  # must not raise
        assert scheduler.requests == []
    finally:
        module.stop()


def test_on_policy_request_passes_unknown_names_through_and_survives_rejection() -> None:
    # Validation of names/actions is the scheduler's job (it reports via
    # policy_state.last_error); the module only guards the JSON shape.
    module = MicroduckSimModule()
    try:
        scheduler = _FakeScheduler(accept=False)
        module._scheduler = scheduler  # type: ignore[assignment]
        module._on_policy_request(json.dumps({"action": "jump", "policy": "moonwalk"}))
        assert scheduler.requests == [("moonwalk", "jump")]
    finally:
        module.stop()


def test_on_policy_request_before_start_is_a_noop() -> None:
    module = MicroduckSimModule()
    try:
        module._on_policy_request(json.dumps({"action": "start", "policy": "walk"}))
    finally:
        module.stop()


# --------------------------------------------------------------- fall detector


class _FakeEngineData:
    def __init__(self) -> None:
        self.qpos = np.array([1.0, 0.0, 0.12, 1.0, 0.0, 0.0, 0.0])


class _FakeEngine:
    def __init__(self) -> None:
        self.data = _FakeEngineData()
        self.model = None


def test_check_fall_is_debounced_and_nudges_toward_origin() -> None:
    module = MicroduckSimModule(auto_stand=True, auto_stand_after=2.0)
    try:
        bank = _FakeBank(gravity_z=0.9)  # lying on its back
        scheduler = _FakeScheduler()
        module._bank = bank  # type: ignore[assignment]
        module._scheduler = scheduler  # type: ignore[assignment]
        stood: list[Any] = []
        module._stand_in_place = stood.append  # type: ignore[method-assign]
        engine = _FakeEngine()

        assert module._check_fall(engine, now=10.0) is False  # timer starts
        assert module._check_fall(engine, now=11.9) is False  # within grace
        assert scheduler.falls == [] and stood == []
        assert module._check_fall(engine, now=12.1) is True  # stood back up
        assert scheduler.falls == [True]
        assert stood == [engine]
        assert engine.data.qpos[0] == pytest.approx(0.75)  # nudged 0.25 m toward origin
        assert module._fallen_since is None

        bank.gravity_z = -1.0  # upright again
        assert module._check_fall(engine, now=12.2) is False
        assert scheduler.falls == [True, False]
    finally:
        module.stop()


def test_check_fall_without_auto_stand_only_notifies() -> None:
    module = MicroduckSimModule(auto_stand=False, auto_stand_after=0.5)
    try:
        scheduler = _FakeScheduler()
        module._bank = _FakeBank(gravity_z=0.2)  # type: ignore[assignment]
        module._scheduler = scheduler  # type: ignore[assignment]
        stood: list[Any] = []
        module._stand_in_place = stood.append  # type: ignore[method-assign]
        engine = _FakeEngine()
        assert module._check_fall(engine, now=1.0) is False
        assert module._check_fall(engine, now=2.0) is False
        assert scheduler.falls == [True] and stood == []
        assert engine.data.qpos[0] == 1.0
    finally:
        module.stop()


def test_check_fall_is_skipped_while_scheduler_suspends_it() -> None:
    module = MicroduckSimModule(auto_stand_after=0.0)
    try:
        scheduler = _FakeScheduler()
        scheduler.suspend_fall_detector = True  # e.g. the roulade is running
        module._bank = _FakeBank(gravity_z=1.0)  # type: ignore[assignment]
        module._scheduler = scheduler  # type: ignore[assignment]
        engine = _FakeEngine()
        assert module._check_fall(engine, now=1.0) is False
        assert module._check_fall(engine, now=5.0) is False
        assert scheduler.falls == [] and module._fallen_since is None
    finally:
        module.stop()


# ---------------------------------------------------------------- chase camera


def test_publish_chase_emits_once_per_rendered_frame() -> None:
    module = MicroduckSimModule(chase_cam=True)
    try:
        frames: dict[str, CameraFrame | None] = {CHASE_CAMERA_NAME: None}

        class _Engine:
            def read_camera(self, name: str) -> CameraFrame | None:
                return frames.get(name)

        engine = _Engine()
        images: list[Any] = []
        module.chase_image.subscribe(images.append)

        module._publish_chase(engine)  # nothing rendered yet
        assert images == []
        frames[CHASE_CAMERA_NAME] = _frame(5.0)
        module._publish_chase(engine)
        module._publish_chase(engine)  # same frame again: no republish
        assert len(images) == 1
        frames[CHASE_CAMERA_NAME] = _frame(5.1)
        module._publish_chase(engine)
        assert len(images) == 2
        image = images[-1]
        assert image.ts == 5.1
        assert image.format == ImageFormat.RGB
        assert image.frame_id == f"{CHASE_CAMERA_NAME}_optical_frame"
        assert np.asarray(image.data).shape == (360, 640, 3)
    finally:
        module.stop()


def test_publish_chase_is_disabled_with_chase_cam_false() -> None:
    module = MicroduckSimModule(chase_cam=False)
    try:

        class _Engine:
            def read_camera(self, name: str) -> CameraFrame:
                raise AssertionError("must not read the chase camera when disabled")

        images: list[Any] = []
        module.chase_image.subscribe(images.append)
        module._publish_chase(_Engine())
        assert images == []
    finally:
        module.stop()


# ------------------------------------------- extra_cameras on MujocoSimModule


class _EngineCapturedError(Exception):
    def __init__(self, kwargs: dict[str, Any]) -> None:
        super().__init__("captured")
        self.kwargs = kwargs


class _NoShm:
    def __init__(self, key: str) -> None:
        self.key = key

    def __getattr__(self, name: str) -> Any:
        return lambda *args, **kwargs: None


def _capture_engine_cameras(
    monkeypatch: pytest.MonkeyPatch, module: MujocoSimModule
) -> list[msm.CameraConfig]:
    """Run ``start()`` up to engine construction and return the camera list."""

    def _fake_engine(**kwargs: Any) -> None:
        raise _EngineCapturedError(kwargs)

    monkeypatch.setattr(msm, "ManipShmWriter", _NoShm)
    monkeypatch.setattr(msm, "MujocoEngine", _fake_engine)
    monkeypatch.setattr(module, "_compose_model", lambda: None)
    with pytest.raises(_EngineCapturedError) as info:
        module.start()
    return list(info.value.kwargs["cameras"])


def test_extra_cameras_default_leaves_camera_list_unchanged(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    kwargs = dict(robot_mjcf="/nonexistent/robot.xml", width=320, height=240, fps=5)
    module = MujocoSimModule(enable_color=True, **kwargs)
    try:
        assert module.config.extra_cameras == {}
        cameras = _capture_engine_cameras(monkeypatch, module)
        assert [(c.name, c.width, c.height, c.fps) for c in cameras] == [
            (module.config.camera_name, 320, 240, 5.0)
        ]
    finally:
        module.stop()


def test_extra_cameras_append_after_the_primary_camera(monkeypatch: pytest.MonkeyPatch) -> None:
    module = MujocoSimModule(
        robot_mjcf="/nonexistent/robot.xml",
        width=320,
        height=240,
        fps=5,
        enable_color=True,
        base_frame_id="trunk_base",
        extra_cameras={
            "chase_camera": (640, 360, 12.0),
            "": (10, 10, 1.0),  # ignored
            "wrist_camera": (1, 1, 1.0),  # same name as the primary: ignored
        },
    )
    try:
        cameras = _capture_engine_cameras(monkeypatch, module)
        assert [(c.name, c.width, c.height, c.fps) for c in cameras] == [
            ("wrist_camera", 320, 240, 5.0),
            ("chase_camera", 640, 360, 12.0),
        ]
        chase = cameras[1]
        assert chase.base_body_name == "trunk_base"
        assert chase.geom_groups is None and chase.max_geom == 10000
    finally:
        module.stop()


def test_extra_cameras_work_without_a_primary_camera(monkeypatch: pytest.MonkeyPatch) -> None:
    module = MujocoSimModule(
        robot_mjcf="/nonexistent/robot.xml",
        enable_color=False,
        enable_depth=False,
        enable_pointcloud=False,
        extra_cameras={"chase_camera": (640, 360, 12.0)},
    )
    try:
        cameras = _capture_engine_cameras(monkeypatch, module)
        assert [c.name for c in cameras] == ["chase_camera"]
    finally:
        module.stop()


def test_microduck_extra_cameras_default_is_empty_until_start() -> None:
    module = MicroduckSimModule()
    try:
        # start() merges the chase camera in; the config itself stays generic.
        assert module.config.extra_cameras == {}
        assert module.config.chase_cam is True
    finally:
        module.stop()


# ------------------------------------------------ MuJoCo composition (-m mujoco)


def _module(**overrides: Any) -> MicroduckSimModule:
    robot_xml = _cache_or_skip()
    kwargs: dict[str, Any] = dict(
        scene_xml=FOUR_ROOM_XML, robot_mjcf=str(robot_xml), headless=True, spawn_xy=(0.0, 0.0)
    )
    kwargs.update(overrides)
    return MicroduckSimModule(**kwargs)


@pytest.mark.mujoco
def test_compose_model_adds_chase_camera_and_ball_to_four_room_scene() -> None:
    mujoco = pytest.importorskip("mujoco")
    module = _module()
    try:
        model = module._compose_model()
        # Trunk free joint stays joint 0 (engine's robot root); ball comes after.
        assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
        assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, 0) == "trunk_base_freejoint"
        ball = model.joint(f"{BALL_BODY}_freejoint")
        assert int(ball.type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        assert model.nu == 14 and model.njnt == 16
        assert model.opt.timestep == pytest.approx(PHYSICS_TIMESTEP)

        cam = model.camera(CHASE_CAMERA_NAME)
        assert int(cam.mode[0]) == int(mujoco.mjtCamLight.mjCAMLIGHT_TRACK)
        assert (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(cam.bodyid[0])) == "trunk_base"
        )
        assert cam.pos == pytest.approx(module.config.chase_cam_offset)
        assert float(cam.fovy[0]) == pytest.approx(module.config.chase_cam_fovy)
        for name, _ in LIDAR_CAMERA_SPECS:
            assert model.camera(name).id >= 0
        assert model.camera("head_camera").id >= 0

        # The POV camera is the one worth asserting on: the MJCF's stock
        # head_camera looks along body -x, i.e. backwards into the duck's own
        # jaw, so the first-person view (and the observe skill) uses ours.
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        pov = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, POV_CAMERA_NAME)
        head = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        assert pov >= 0
        # MuJoCo cameras look along their own -z.
        forward = -data.cam_xmat[pov].reshape(3, 3)[:, 2]
        assert forward[0] == pytest.approx(1.0, abs=1e-6)  # body +x, where it walks
        assert -data.cam_xmat[head].reshape(3, 3)[:, 2][0] == pytest.approx(-1.0, abs=1e-6)
        # Same eye position as the stock camera, just facing the other way.
        assert data.cam_xpos[pov] == pytest.approx(data.cam_xpos[head], abs=2e-3)

        assert model.vis.global_.offwidth >= 1280
        assert model.vis.global_.offheight >= 720
        assert model.vis.global_.offwidth >= module.config.chase_cam_size[0]
        assert model.vis.global_.offheight >= module.config.chase_cam_size[1]

        # The scene's lights are kept but stop casting shadows by default.
        assert model.nlight == 2
        assert not model.light_castshadow.any()
    finally:
        module.stop()


@pytest.mark.mujoco
def test_compose_model_omits_chase_camera_and_ball_when_disabled() -> None:
    mujoco = pytest.importorskip("mujoco")
    module = _module(chase_cam=False, ball_body="", cast_shadows=True)
    try:
        model = module._compose_model()
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CHASE_CAMERA_NAME) == -1
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{BALL_BODY}_freejoint") == -1
        assert model.njnt == 15 and model.nu == 14
        # cast_shadows=True leaves the scene's shadow-casting lights alone.
        assert model.nlight == 2 and model.light_castshadow.all()
    finally:
        module.stop()


@pytest.mark.mujoco
def test_compose_model_raises_offscreen_buffer_to_chase_cam_size() -> None:
    mujoco = pytest.importorskip("mujoco")
    # No scene file: MjSpec's offscreen buffer defaults to 640x480.
    module = _module(scene_xml=None, chase_cam_size=(1024, 576), chase_cam_fovy=45.0)
    try:
        model = module._compose_model()
        assert model.vis.global_.offwidth >= 1024
        assert model.vis.global_.offheight >= 576
        assert float(model.camera(CHASE_CAMERA_NAME).fovy[0]) == pytest.approx(45.0)
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{BALL_BODY}_freejoint") >= 0
    finally:
        module.stop()


class _ModelEngine:
    """Just enough engine for ``_spawn_ball``: a model, its data, and stop()."""

    def __init__(self, model: Any) -> None:
        import mujoco

        self.model = model
        self.data = mujoco.MjData(model)

    def disconnect(self) -> None:
        pass


@pytest.mark.mujoco
def test_spawn_ball_places_ball_ahead_of_the_trunk_in_its_yaw_frame() -> None:
    pytest.importorskip("mujoco")
    module = _module()
    try:
        model = module._compose_model()
        engine = _ModelEngine(model)
        data = engine.data
        ball = model.joint(f"{BALL_BODY}_freejoint")
        module._ball_qpos_adr = int(ball.qposadr[0])
        module._ball_qvel_adr = int(ball.dofadr[0])
        module._bank = _FakeBank(root_qpos_adr=0)  # type: ignore[assignment]
        module._engine = engine  # type: ignore[assignment]

        # Trunk at (1, 2) facing +y (yaw 90 deg); ball moving before the spawn.
        data.qpos[0:3] = (1.0, 2.0, 0.12)
        data.qpos[3:7] = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
        data.qvel[module._ball_qvel_adr : module._ball_qvel_adr + 6] = 3.0
        module._spawn_ball(0.09, 0.042)

        adr = module._ball_qpos_adr
        assert data.qpos[adr : adr + 3] == pytest.approx([1.0 - 0.042, 2.0 + 0.09, BALL_RADIUS])
        assert data.qpos[adr + 3 : adr + 7] == pytest.approx([1.0, 0.0, 0.0, 0.0])
        assert np.all(data.qvel[module._ball_qvel_adr : module._ball_qvel_adr + 6] == 0.0)
        # mj_forward ran: the ball body's world position reflects the new qpos.
        assert data.xpos[model.body(BALL_BODY).id][:2] == pytest.approx([0.958, 2.09])
    finally:
        module.stop()


@pytest.mark.mujoco
def test_spawn_ball_is_a_noop_without_a_ball_joint() -> None:
    pytest.importorskip("mujoco")
    module = _module(ball_body="")
    try:
        engine = _ModelEngine(module._compose_model())
        before = engine.data.qpos.copy()
        module._bank = _FakeBank(root_qpos_adr=0)  # type: ignore[assignment]
        module._engine = engine  # type: ignore[assignment]
        module._spawn_ball(0.09, 0.042)
        assert np.array_equal(engine.data.qpos, before)
    finally:
        module.stop()
