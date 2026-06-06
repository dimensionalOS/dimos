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

from pathlib import Path
import threading
import time
from typing import cast
from unittest.mock import MagicMock, patch

import pytest

from dimos.manipulation.viser_panel.module import (
    GOAL_ROBOT_FEASIBLE_COLOR,
    GOAL_ROBOT_FEASIBLE_OPACITY,
    GOAL_ROBOT_INFEASIBLE_COLOR,
    GOAL_ROBOT_INFEASIBLE_OPACITY,
    GOAL_ROBOT_MESH_COLOR,
    ViserManipulationPanelConfig,
    ViserManipulationPanelModule,
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


class FakeViserUrdf:
    def __init__(self) -> None:
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
    panel = _make_panel()
    cast("ViserManipulationPanelConfig", panel.config).preview_request_timeout = 0.01
    release = threading.Event()

    def slow_preview() -> dict[str, object]:
        release.wait(timeout=1.0)
        return {"success": True}

    result = panel._call_preview_with_timeout(slow_preview, "IK_TIMEOUT")
    release.set()
    time.sleep(0.02)

    assert result["success"] is False
    assert result["status"] == "IK_TIMEOUT"
    assert "timed out" in str(result["message"])


def test_panel_cartesian_preview_uses_timeout_wrapper():
    panel = _make_panel()
    panel._client = MagicMock()
    panel._client.solve_ik_preview.return_value = {"success": True, "collision_free": True}
    request = PreviewRequest(1, "cartesian", "arm", pose=Pose.__new__(Pose))

    with patch.object(
        panel, "_call_preview_with_timeout", return_value={"success": False}
    ) as timeout:
        result = panel._handle_preview_request(request)

    assert result == {"success": False}
    timeout.assert_called_once()


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


def test_panel_applies_feasible_preview_and_updates_target_color():
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
    assert panel._handles["ee_control"].color == (0, 180, 255)
    assert panel._urdfs["arm:ghost"].color == (0, 180, 255)


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


def test_panel_renders_plan_path_with_viser_line_segment_shape():
    panel = _make_panel()
    path = [_joint_state([0.0]), _joint_state([0.1]), _joint_state([0.2])]

    panel._render_plan_path(path)

    assert panel._server is not None
    assert panel._server.scene.line_segments[0]["points"] == [
        [[0.0, 0.0, 0.02], [1.0, 0.1, 0.02]],
        [[1.0, 0.1, 0.02], [2.0, 0.2, 0.02]],
    ]


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


def test_panel_creates_goal_robot_as_transparent_colored_overlay():
    panel = _make_panel()
    panel._urdfs = {}
    created: list[dict[str, object]] = []

    class CapturingViserUrdf:
        def __init__(self, _server: object, _path: object, **kwargs: object) -> None:
            created.append(kwargs)

    panel._viser_urdf = CapturingViserUrdf

    with patch.object(panel, "_prepared_urdf_path", return_value=Path("robot.urdf")):
        panel._ensure_scene_nodes("arm", {"model_path": "robot.xacro"})

    assert created == [
        {"root_node_name": "/robots/arm/current", "mesh_color_override": None},
        {"root_node_name": "/targets/arm/ghost", "mesh_color_override": GOAL_ROBOT_MESH_COLOR},
    ]


def test_panel_updates_goal_robot_mesh_color_for_feasibility():
    panel = _make_panel()
    ghost = FakeViserUrdf()
    panel._urdfs["arm:ghost"] = ghost

    panel._set_target_visual_state(True)

    assert all(mesh.color == GOAL_ROBOT_FEASIBLE_COLOR for mesh in ghost._meshes)
    assert all(mesh.opacity == GOAL_ROBOT_FEASIBLE_OPACITY for mesh in ghost._meshes)

    panel._set_target_visual_state(False)

    assert all(mesh.color == GOAL_ROBOT_INFEASIBLE_COLOR for mesh in ghost._meshes)
    assert all(mesh.opacity == GOAL_ROBOT_INFEASIBLE_OPACITY for mesh in ghost._meshes)


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
    panel = _make_panel()
    path = [_joint_state([0.0, 0.0]), _joint_state([1.0, 2.0])]

    frames = panel._interpolate_joint_path(path, duration=1.0, fps=2.0)

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


def test_panel_preview_does_not_trigger_backend_drake_preview():
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
