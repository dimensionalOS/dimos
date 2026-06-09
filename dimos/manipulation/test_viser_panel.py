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

from __future__ import annotations

import threading
import time
from typing import cast
from unittest.mock import MagicMock, patch

import pytest

from dimos.manipulation.viser_panel.animation import interpolate_joint_path
from dimos.manipulation.viser_panel.backend import PanelBackend
from dimos.manipulation.viser_panel.module import (
    ViserManipulationPanelConfig,
    ViserManipulationPanelModule,
)
from dimos.manipulation.viser_panel.scene import (
    PanelScene,
)
from dimos.manipulation.viser_panel.state import (
    ActionStatus,
    BackendConnectionStatus,
    FeasibilityStatus,
    PanelPlanState,
    PanelRuntime,
    PanelSession,
    PlanStatus,
    PreviewRequest,
    PreviewWorker,
    TargetStatus,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def _joint_state(position: list[float], name: list[str] | None = None) -> JointState:
    joint_state = JointState.__new__(JointState)
    joint_state.ts = time.time()
    joint_state.frame_id = ""
    joint_state.name = name or []
    joint_state.position = position
    joint_state.velocity = []
    joint_state.effort = []
    return joint_state


def _unit_quaternion() -> Quaternion:
    quaternion = Quaternion.__new__(Quaternion)
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = 0.0
    quaternion.w = 1.0
    return quaternion


class FakeHandle:
    def __init__(self, value=None):
        self.value = value
        self.disabled = False
        self.color = None
        self.opacity = None
        self.position = (0.0, 0.0, 0.0)
        self.wxyz = (1.0, 0.0, 0.0, 0.0)
        self.removed = False

    def remove(self):
        self.removed = True


class FakeScene:
    def __init__(self) -> None:
        self.line_segments: list[dict[str, object]] = []

    def add_line_segments(self, _name: str, **kwargs: object) -> FakeHandle:
        self.line_segments.append(kwargs)
        return FakeHandle()


class FakeServer:
    def __init__(self) -> None:
        self.scene = FakeScene()


class FakeViserUrdf:
    def __init__(self, *_args: object, **_kwargs: object) -> None:
        self._urdf = type(
            "FakeUrdf", (), {"actuated_joint_names": ["joint1", "joint2", "drive_joint"]}
        )()
        self.cfg: list[float] | None = None
        self._meshes = [FakeHandle(), FakeHandle()]
        self.cfg_history: list[list[float]] = []

    def update_cfg(self, cfg: list[float]) -> None:
        self.cfg = cfg
        self.cfg_history.append(cfg)


def _make_panel() -> ViserManipulationPanelModule:
    with patch.object(ViserManipulationPanelModule, "__init__", lambda self: None):
        panel = ViserManipulationPanelModule.__new__(ViserManipulationPanelModule)
    panel.config = ViserManipulationPanelConfig()
    panel.config.allow_plan_execute = True
    panel.session = PanelSession(selected_robot="arm")
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel._client = MagicMock()
    panel._server = type("FakeServer", (), {"scene": FakeScene()})()
    panel._urdfs = {"arm:ghost": FakeHandle()}
    panel._handles = {
        "status": FakeHandle(),
        "error": FakeHandle(),
        "feasibility": FakeHandle(),
        "plan": FakeHandle(),
        "preview": FakeHandle(),
        "execute": FakeHandle(),
        "ee_control": FakeHandle(),
    }
    panel._joint_sliders = {}
    panel._viser_urdf = None
    panel._lock = threading.RLock()
    return panel


def test_panel_session_marks_fresh_plan_stale_on_new_target_sequence():
    session = PanelSession(selected_robot="arm")
    session.plan_state = PanelPlanState(status=PlanStatus.FRESH)

    sequence_id = session.next_sequence_id()

    assert sequence_id == 1
    assert session.feasibility.status == FeasibilityStatus.UNKNOWN
    assert session.plan_state.status == PlanStatus.STALE


def test_panel_session_requires_fresh_matching_plan_for_execution():
    session = PanelSession(
        selected_robot="arm",
        runtime=PanelRuntime.RUNNING,
        backend_status=BackendConnectionStatus.READY,
        target_status=TargetStatus.FEASIBLE,
        manipulation_state="IDLE",
        current_joints=[0.0, 0.01],
    )
    session.feasibility.status = FeasibilityStatus.FEASIBLE
    session.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[0.0, 0.0],
    )

    assert session.can_execute(0.02)
    session.current_joints = [0.0, 0.1]
    assert not session.can_execute(0.02)


def test_preview_worker_coalesces_to_latest_request():
    handled: list[int] = []
    applied: list[int] = []

    def handle(request: PreviewRequest) -> dict[str, object]:
        handled.append(request.sequence_id)
        return {"success": True}

    def apply(request: PreviewRequest, _result: dict[str, object]) -> None:
        applied.append(request.sequence_id)

    worker = PreviewWorker(handle, apply, debounce_seconds=0.02)
    worker.start()
    worker.submit(PreviewRequest(1, "joints", "arm", joints=_joint_state([0.0])))
    worker.submit(PreviewRequest(2, "joints", "arm", joints=_joint_state([1.0])))
    time.sleep(0.12)
    worker.stop()

    assert handled[-1:] == [2]
    assert applied[-1:] == [2]


def test_panel_ignores_stale_preview_result():
    panel = _make_panel()
    panel.session.latest_sequence_id = 2

    panel._apply_preview_result(PreviewRequest(1, "joints", "arm"), {"success": True})

    assert panel.session.feasibility.status == FeasibilityStatus.UNKNOWN


def test_panel_preview_request_times_out_without_blocking_worker_forever():
    backend = PanelBackend(
        module_ref=None,
        module_rpc=None,
        remote_factory=MagicMock(),
        timeout_seconds=lambda: 0.01,
    )
    release = threading.Event()

    def slow_preview() -> dict[str, object]:
        release.wait(timeout=1.0)
        return {"success": True}

    result = backend.call_preview_with_timeout(slow_preview, "IK_TIMEOUT")
    release.set()
    time.sleep(0.02)

    assert result["success"] is False
    assert result["status"] == "IK_TIMEOUT"
    assert "timed out" in str(result["message"])


def test_panel_cartesian_preview_uses_timeout_wrapper():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.evaluate_pose_target.return_value = {"success": True, "collision_free": True}
    request = PreviewRequest(1, "cartesian", "arm", pose=Pose.__new__(Pose))

    with patch.object(
        panel, "_call_preview_with_timeout", return_value={"success": False}
    ) as timeout:
        result = panel._handle_preview_request(request)

    assert result == {"success": False}
    timeout.assert_called_once()
    panel._client.evaluate_pose_target.assert_not_called()
    panel._client.solve_ik_preview.assert_not_called()


def test_panel_cartesian_preview_routes_to_target_evaluation_api():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.evaluate_pose_target.return_value = {"success": True, "collision_free": True}
    request = PreviewRequest(1, "cartesian", "arm", pose=Pose.__new__(Pose))

    result = panel._handle_preview_request(request)

    assert result == {"success": True, "collision_free": True}
    panel._client.evaluate_pose_target.assert_called_once_with(request.pose, "arm")
    panel._client.solve_ik_preview.assert_not_called()


def test_panel_refresh_reports_disconnected_state_when_rpc_unavailable():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.list_robots.side_effect = RuntimeError("rpc unavailable")

    snapshot = panel.refresh_panel_state()

    assert snapshot["connected"] is False
    assert snapshot["module_state"] == "DISCONNECTED"
    assert snapshot["can_plan"] is False
    assert snapshot["can_execute"] is False


def test_panel_waits_when_default_robot_is_not_reported_yet():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.list_robots.return_value = []

    snapshot = panel.refresh_panel_state()

    assert snapshot["connected"] is True
    assert snapshot["module_state"] == "WAITING_FOR_ROBOT"
    assert snapshot["backend_status"] == "waiting_for_robot"
    assert snapshot["can_plan"] is False
    assert panel._handles["status"].value == "WAITING_FOR_ROBOT"


def test_panel_waits_when_manipulation_module_is_not_built_yet():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.list_robots.side_effect = AttributeError("_robots")

    snapshot = panel.refresh_panel_state()

    assert snapshot["connected"] is True
    assert snapshot["module_state"] == "WAITING_FOR_ROBOT"
    assert snapshot["backend_status"] == "waiting_for_robot"
    assert "_robots" in snapshot["error"]


def test_panel_recreates_stale_rpc_client_after_timeout():
    panel = _make_panel()
    stale_client = MagicMock()
    fresh_client = MagicMock()
    stale_client.list_robots.side_effect = TimeoutError()
    fresh_client.list_robots.return_value = []
    panel._client = stale_client

    with patch(
        "dimos.manipulation.viser_panel.module.RPCClient.remote",
        return_value=fresh_client,
    ):
        snapshot = panel.refresh_panel_state()

    stale_client.stop_rpc_client.assert_called_once()
    assert panel._client is fresh_client
    assert snapshot["connected"] is True
    assert snapshot["module_state"] == "WAITING_FOR_ROBOT"
    assert snapshot["backend_status"] == "waiting_for_robot"


def test_panel_waits_when_robot_world_is_not_finalized_yet():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.list_robots.return_value = ["arm"]
    panel._client.get_robot_info.return_value = {"joint_names": ["j1"]}
    panel._client.get_current_joints.return_value = [0.0]
    panel._client.get_ee_pose.side_effect = RuntimeError("World must be finalized first")

    snapshot = panel.refresh_panel_state()

    assert snapshot["connected"] is True
    assert snapshot["module_state"] == "WAITING_FOR_ROBOT"
    assert snapshot["backend_status"] == "waiting_for_robot"
    assert "finalized" in snapshot["error"]


def test_panel_fails_fast_when_viser_urdf_dependency_is_missing():
    panel = _make_panel()

    with patch("importlib.import_module", side_effect=ImportError("missing yourdfpy")):
        with pytest.raises(ModuleNotFoundError, match="yourdfpy"):
            panel._import_viser_urdf()


def test_panel_applies_feasible_preview_to_targets():
    panel = _make_panel()
    panel.session.latest_sequence_id = 1
    panel.session.robot_info = {"joint_names": ["j1", "j2"]}
    joint_state = _joint_state([0.1, 0.2], ["j1", "j2"])

    panel._apply_preview_result(
        PreviewRequest(1, "cartesian", "arm"),
        {"success": True, "collision_free": True, "joint_state": joint_state},
    )

    assert panel.session.feasibility.status == FeasibilityStatus.FEASIBLE
    assert panel.session.target_status == TargetStatus.FEASIBLE
    assert panel.session.joint_target == [0.1, 0.2]


def test_panel_execute_enabled_after_fresh_completed_plan():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.selected_robot = "arm"
    panel.session.manipulation_state = "COMPLETED"
    panel.session.current_joints = [0.0, 0.0]
    panel.session.feasibility.status = FeasibilityStatus.FEASIBLE
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[0.0, 0.0],
    )

    panel._update_gui_state()

    assert not panel._handles["execute"].disabled


def test_panel_execute_requires_operator_launch_opt_in():
    panel = _make_panel()
    cast("ViserManipulationPanelConfig", panel.config).allow_plan_execute = False
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.manipulation_state = "IDLE"
    panel.session.current_joints = [0.0]
    panel.session.feasibility.status = FeasibilityStatus.FEASIBLE
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[0.0],
    )

    panel._update_gui_state()

    assert panel._handles["execute"].disabled


def test_panel_execute_operation_gate_does_not_mutate_action_status():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.selected_robot = "arm"
    panel.session.manipulation_state = "COMPLETED"
    panel.session.current_joints = [0.0]
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.action_status = ActionStatus.EXECUTING
    panel.session.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[0.0],
    )

    assert panel._can_execute_for_operation()
    assert panel.session.action_status == ActionStatus.EXECUTING


def test_panel_renders_plan_path_with_viser_line_segment_shape():
    session = PanelSession(selected_robot="arm")
    handles: dict[str, object] = {}
    server = FakeServer()
    scene = PanelScene(server, session, handles, {}, None)
    path = [_joint_state([0.0]), _joint_state([0.1]), _joint_state([0.2])]
    poses = [
        PoseStamped(position=Vector3(0.1, 0.2, 0.3), orientation=_unit_quaternion()),
        PoseStamped(position=Vector3(0.4, 0.5, 0.6), orientation=_unit_quaternion()),
        PoseStamped(position=Vector3(0.7, 0.8, 0.9), orientation=_unit_quaternion()),
    ]

    scene.render_plan_path(path, poses)

    points = cast("list[list[list[float]]]", server.scene.line_segments[0]["points"])
    assert len(points) == len(poses) - 1
    assert points[0][0] == [0.1, 0.2, 0.3]


def test_panel_backend_rejects_new_preview_when_previous_timed_out():
    backend = PanelBackend(
        module_ref=None,
        module_rpc=None,
        remote_factory=MagicMock(),
        timeout_seconds=lambda: 0.01,
    )
    release = threading.Event()
    started = threading.Event()
    active = 0

    def slow_preview() -> dict[str, object]:
        nonlocal active
        active += 1
        started.set()
        release.wait(timeout=1.0)
        active -= 1
        return {"success": True}

    first = backend.call_preview_with_timeout(slow_preview, "IK_TIMEOUT")
    assert first["status"] == "IK_TIMEOUT"
    assert started.is_set()

    second = backend.call_preview_with_timeout(slow_preview, "IK_TIMEOUT")

    release.set()
    assert backend._preview_thread is not None
    backend._preview_thread.join(timeout=1.0)
    assert second["success"] is False
    assert second["status"] == "PREVIEW_BUSY"
    assert active == 0


def test_panel_snapshot_does_not_force_refresh():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.manipulation_state = "IDLE"

    snapshot = panel.get_panel_snapshot()

    assert snapshot["connected"] is True
    assert snapshot["module_state"] == "IDLE"


def test_panel_updates_viser_urdf_with_matching_named_joints():
    panel = _make_panel()
    panel.session.robot_info = {"joint_names": ["arm/joint1", "arm/joint2", "arm/gripper"]}
    urdf = FakeViserUrdf()

    panel._set_urdf_joints(urdf, [0.1, 0.2, 0.3])

    assert urdf.cfg == [0.1, 0.2, 0.0]


def test_panel_creates_current_and_goal_robot_scene_nodes():
    panel = _make_panel()
    panel._urdfs = {}
    panel._viser_urdf = FakeViserUrdf

    with patch.object(panel, "_prepared_urdf_path", return_value="robot.urdf"):
        panel._ensure_scene_nodes("arm", {"model_path": "robot.xacro"})

    assert "arm:current" in panel._urdfs
    assert "arm:ghost" in panel._urdfs


def test_panel_updates_goal_robot_visual_state_for_feasibility():
    panel = _make_panel()
    ghost = FakeViserUrdf()
    panel._urdfs["arm:ghost"] = ghost

    panel._set_target_visual_state(True)
    feasible_colors = [mesh.color for mesh in ghost._meshes]
    feasible_opacities = [mesh.opacity for mesh in ghost._meshes]

    assert all(color is not None for color in feasible_colors)
    assert all(opacity is not None for opacity in feasible_opacities)

    panel._set_target_visual_state(False)

    assert [mesh.color for mesh in ghost._meshes] != feasible_colors
    assert [mesh.opacity for mesh in ghost._meshes] != feasible_opacities


def test_panel_plan_runs_while_plan_operation_is_marked_in_flight():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.selected_robot = "arm"
    panel.session.manipulation_state = "IDLE"
    panel.session.feasibility.status = FeasibilityStatus.FEASIBLE
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.joint_target = [0.1, 0.2]
    panel.session.current_joints = [0.0, 0.0]
    panel.session.robot_info = {"joint_names": ["j1", "j2"]}
    client = panel._client
    assert client is not None
    client.plan_to_joints.return_value = True
    client.get_planned_path.return_value = [_joint_state([0.0]), _joint_state([0.1])]

    panel._plan()

    client.plan_to_joints.assert_called_once()
    assert panel.session.plan_state.status == PlanStatus.FRESH


def test_panel_plan_uses_client_snapshot_when_client_resets_mid_operation():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.selected_robot = "arm"
    panel.session.manipulation_state = "IDLE"
    panel.session.feasibility.status = FeasibilityStatus.FEASIBLE
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.joint_target = [0.1, 0.2]
    panel.session.current_joints = [0.0, 0.0]
    panel.session.robot_info = {"joint_names": ["j1", "j2"]}
    client = panel._client
    assert client is not None
    client.get_planned_path.return_value = [_joint_state([0.0]), _joint_state([0.1])]

    def reset_client_during_plan(_target: JointState, _robot: str) -> bool:
        panel._client = None
        return True

    client.plan_to_joints.side_effect = reset_client_during_plan

    panel._plan()

    client.get_planned_path.assert_called_once_with("arm")
    assert panel.session.plan_state.status == PlanStatus.FRESH


def test_panel_state_axes_keep_action_separate_from_plan_state():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.manipulation_state = "IDLE"
    panel.session.target_status = TargetStatus.FEASIBLE

    assert panel.session.can_plan()
    panel.session.action_status = ActionStatus.PREVIEWING
    assert not panel.session.can_plan()
    assert panel.session.plan_state.status == PlanStatus.NONE


def test_panel_preview_animates_viser_ghost_path():
    panel = _make_panel()
    cast("ViserManipulationPanelConfig", panel.config).preview_duration = 0.0
    panel.session.selected_robot = "arm"
    panel.session.robot_info = {"joint_names": ["arm/joint1", "arm/joint2", "arm/gripper"]}
    ghost = FakeViserUrdf()
    panel._urdfs["arm:ghost"] = ghost
    client = panel._client
    assert client is not None
    client.get_planned_path.return_value = [_joint_state([0.1, 0.2, 0.3])]

    panel._preview()

    client.preview_path.assert_not_called()
    assert ghost.cfg == [0.1, 0.2, 0.0]
    assert "Previewing" in panel.session.error


def test_panel_interpolates_sparse_preview_path():
    path = [_joint_state([0.0, 0.0]), _joint_state([1.0, 2.0])]

    frames = interpolate_joint_path(path, duration=1.0, fps=2.0)

    assert frames == [[0.0, 0.0], [0.5, 1.0], [1.0, 2.0]]


def test_panel_preview_animates_interpolated_frames():
    panel = _make_panel()
    cast("ViserManipulationPanelConfig", panel.config).preview_duration = 1.0
    cast("ViserManipulationPanelConfig", panel.config).preview_fps = 2.0
    panel.session.selected_robot = "arm"
    panel.session.robot_info = {"joint_names": ["arm/joint1", "arm/joint2"]}
    ghost = FakeViserUrdf()
    panel._urdfs["arm:ghost"] = ghost

    with patch("dimos.manipulation.viser_panel.module.time.sleep"):
        panel._animate_ghost_path([_joint_state([0.0, 0.0]), _joint_state([1.0, 2.0])], 1.0)

    assert ghost.cfg_history == [[0.0, 0.0, 0.0], [0.5, 1.0, 0.0], [1.0, 2.0, 0.0]]


def test_panel_target_sync_does_not_overwrite_ghost_during_preview():
    panel = _make_panel()
    panel.session.selected_robot = "arm"
    panel.session.robot_info = {"joint_names": ["arm/joint1", "arm/joint2", "arm/gripper"]}
    panel.session.joint_target = [0.9, 0.8, 0.7]
    panel.session.action_status = ActionStatus.PREVIEWING
    ghost = FakeViserUrdf()
    ghost.update_cfg([0.1, 0.2, 0.0])
    panel._urdfs["arm:ghost"] = ghost

    panel._sync_controls_from_targets()

    assert ghost.cfg == [0.1, 0.2, 0.0]


def test_panel_initializes_target_from_current_robot_state():
    panel = _make_panel()
    panel.session.selected_robot = "arm"
    panel.session.robot_info = {"joint_names": ["arm/joint1", "arm/joint2", "arm/gripper"]}
    panel.session.current_joints = [0.4, 0.5, 0.6]
    panel.session.current_ee_pose = panel._make_pose((1.0, 2.0, 3.0), (0.1, 0.2, 0.3, 0.4))
    ghost = FakeViserUrdf()
    panel._urdfs["arm:ghost"] = ghost

    panel._initialize_target_from_current_state()

    assert panel.session.cartesian_target is panel.session.current_ee_pose
    assert panel.session.joint_target == [0.4, 0.5, 0.6]
    assert panel._handles["ee_control"].position == (1.0, 2.0, 3.0)
    assert panel._handles["ee_control"].wxyz == (0.4, 0.1, 0.2, 0.3)
    assert ghost.cfg == [0.4, 0.5, 0.0]


def test_panel_execute_runs_while_execute_operation_is_marked_in_flight():
    panel = _make_panel()
    panel.session.runtime = PanelRuntime.RUNNING
    panel.session.backend_status = BackendConnectionStatus.READY
    panel.session.selected_robot = "arm"
    panel.session.manipulation_state = "COMPLETED"
    panel.session.current_joints = [0.0, 0.0]
    panel.session.feasibility.status = FeasibilityStatus.FEASIBLE
    panel.session.target_status = TargetStatus.FEASIBLE
    panel.session.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[0.0, 0.0],
    )
    panel.session.action_status = ActionStatus.EXECUTING
    client = panel._client
    assert client is not None
    client.execute.return_value = True

    panel._execute()

    client.execute.assert_called_once_with("arm")
    assert panel.session.plan_state.status == PlanStatus.EXECUTING
    assert panel.session.error == "Execution accepted"
