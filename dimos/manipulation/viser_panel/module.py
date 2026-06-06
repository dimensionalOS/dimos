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

from collections.abc import Sequence
import importlib
import threading
import time
import traceback
from typing import Any, cast
import webbrowser

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.viser_panel.animation import interpolate_joint_path
from dimos.manipulation.viser_panel.backend import PanelBackend
from dimos.manipulation.viser_panel.controller import PanelController
from dimos.manipulation.viser_panel.gui import PanelGui
from dimos.manipulation.viser_panel.scene import PanelScene
from dimos.manipulation.viser_panel.state import (
    ActionStatus,
    BackendConnectionStatus,
    FeasibilityStatus,
    PanelRuntime,
    PanelSession,
    PlanStatus,
    PreviewRequest,
    PreviewWorker,
    TargetStatus,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

VISER_INSTALL_HINT = (
    "Install the optional panel dependencies with: uv sync --extra manipulation-viser"
)
VISER_URDF_INSTALL_HINT = (
    "Viser URDF support requires yourdfpy. Install it with: uv sync --extra manipulation-viser"
)


class ViserManipulationPanelConfig(ModuleConfig):
    host: str = "127.0.0.1"
    port: int = 8095
    poll_hz: float = 5.0
    preview_duration: float = 3.0
    open_browser: bool = False
    default_robot: str | None = None
    preview_debounce_seconds: float = 0.05
    preview_request_timeout: float = 5.0
    preview_fps: float = 30.0
    current_match_tolerance: float = 0.02
    allow_plan_execute: bool = False


class ViserManipulationPanelModule(Module):
    dedicated_worker = True
    manipulation: ManipulationModule

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.session = PanelSession(selected_robot=self.panel_config.default_robot)
        self._client: Any | None = None
        self._server: Any | None = None
        self._viser_urdf: Any | None = None
        self._urdfs: dict[str, Any] = {}
        self._handles: dict[str, Any] = {}
        self._joint_sliders: dict[str, Any] = {}
        self._backend: PanelBackend | None = None
        self._gui: PanelGui | None = None
        self._scene: PanelScene | None = None
        self._controller: PanelController | None = None
        self._stop_event = threading.Event()
        self._poll_thread: threading.Thread | None = None
        self._preview_worker = PreviewWorker(
            self._handle_preview_request,
            self._apply_preview_result,
            self.panel_config.preview_debounce_seconds,
        )
        self._lock = threading.RLock()

    @property
    def panel_config(self) -> ViserManipulationPanelConfig:
        return cast("ViserManipulationPanelConfig", self.config)

    def _ensure_backend(self) -> PanelBackend:
        backend = getattr(self, "_backend", None)
        if backend is None:
            backend = PanelBackend(
                module_ref=getattr(self, "manipulation", None),
                module_rpc=getattr(self, "rpc", None),
                remote_factory=RPCClient.remote,
                timeout_seconds=lambda: self.panel_config.preview_request_timeout,
            )
            self._backend = backend
        if getattr(self, "_client", None) is not None and backend.client is None:
            backend.client = self._client
        return backend

    def _ensure_gui(self) -> PanelGui:
        gui = getattr(self, "_gui", None)
        if gui is None:
            gui = PanelGui(self._server, self.session, self._handles, self._joint_sliders)
            self._gui = gui
        else:
            gui.server = self._server
        return gui

    def _ensure_scene(self) -> PanelScene:
        scene = getattr(self, "_scene", None)
        if scene is None:
            scene = PanelScene(
                self._server,
                self.session,
                self._handles,
                self._urdfs,
                self._viser_urdf,
            )
            self._scene = scene
        else:
            scene.server = self._server
            scene.viser_urdf = self._viser_urdf
        return scene

    def _ensure_controller(self) -> PanelController:
        controller = getattr(self, "_controller", None)
        if controller is None:
            controller = PanelController(self)
            self._controller = controller
        return controller

    @rpc
    def start(self) -> None:
        super().start()
        self.session.runtime = PanelRuntime.STARTING
        viser = self._import_viser()
        self._viser_urdf = self._import_viser_urdf()
        self._reset_manipulation_client()
        self._server = viser.ViserServer(host=self.panel_config.host, port=self.panel_config.port)
        self._build_gui()
        self._preview_worker.start()
        self._stop_event.clear()
        self.session.runtime = PanelRuntime.RUNNING
        self.session.backend_status = BackendConnectionStatus.CONNECTING
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        url = f"http://{self.panel_config.host}:{self.panel_config.port}"
        logger.info(f"Viser manipulation panel: {url}")
        if self.panel_config.open_browser:
            webbrowser.open_new_tab(url)

    @rpc
    def stop(self) -> None:
        self.session.runtime = PanelRuntime.STOPPING
        self._stop_event.set()
        self._preview_worker.stop()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        if self._client is not None:
            self._ensure_backend().close()
            self._client = None
        if self._server is not None and hasattr(self._server, "stop"):
            self._server.stop()
        self.session.runtime = PanelRuntime.STOPPED
        super().stop()

    def _import_viser(self) -> Any:
        try:
            viser = importlib.import_module("viser")
        except ModuleNotFoundError as e:
            raise ModuleNotFoundError(VISER_INSTALL_HINT) from e
        return viser

    def _import_viser_urdf(self) -> Any:
        try:
            viser_extras = importlib.import_module("viser.extras")
        except (ImportError, ModuleNotFoundError) as e:
            raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
        return viser_extras.ViserUrdf

    def _build_gui(self) -> None:
        self._ensure_gui().build(
            on_robot=self._select_robot,
            on_preset=self._apply_preset,
            on_plan=lambda: self._run_operation("Plan", self._plan),
            on_preview=lambda: self._run_operation("Preview", self._preview),
            on_execute=lambda: self._run_operation("Execute", self._execute),
            on_cancel=lambda: self._run_operation("Cancel", self._cancel),
            on_clear_plan=lambda: self._run_operation("Clear Plan", self._clear_plan),
        )

    def _wire_static_callbacks(self) -> None:
        if not self._handles:
            return
        self._handles["robot"].on_update(lambda event: self._select_robot(str(event.target.value)))
        self._handles["preset"].on_update(lambda event: self._apply_preset(str(event.target.value)))
        self._handles["plan"].on_click(lambda _: self._run_operation("Plan", self._plan))
        self._handles["preview"].on_click(lambda _: self._run_operation("Preview", self._preview))
        self._handles["execute"].on_click(lambda _: self._run_operation("Execute", self._execute))
        self._handles["cancel"].on_click(lambda _: self._run_operation("Cancel", self._cancel))
        self._handles["clear_plan"].on_click(
            lambda _: self._run_operation("Clear Plan", self._clear_plan)
        )

    def _poll_loop(self) -> None:
        interval = 1.0 / max(self.panel_config.poll_hz, 0.1)
        while not self._stop_event.is_set():
            self.refresh_panel_state()
            self._stop_event.wait(interval)

    def _reset_manipulation_client(self) -> None:
        self._backend = None
        self._client = self._ensure_backend().reset_client()

    def _list_robots(self) -> list[str]:
        robots = self._ensure_backend().list_robots()
        self._client = self._ensure_backend().client
        return robots

    @rpc
    def refresh_panel_state(self) -> dict[str, Any]:
        return self._ensure_controller().refresh_panel_state()

    def _refresh_panel_state_impl(self) -> dict[str, Any]:
        with self._lock:
            try:
                try:
                    robots = self._list_robots()
                except AttributeError as e:
                    self._mark_waiting_for_robot(str(e))
                    self._update_gui_state()
                    return self._snapshot()
                client = self._client
                if client is None:
                    raise RuntimeError("Manipulation RPC client is not connected")
                self.session.backend_status = BackendConnectionStatus.READY
                if robots and self.session.selected_robot not in robots:
                    self._select_robot(robots[0])
                self._update_robot_dropdown(robots)
                if not robots:
                    self._mark_waiting_for_robot("No robots reported by ManipulationModule yet")
                elif self.session.selected_robot:
                    robot = self.session.selected_robot
                    self.session.robot_info = client.get_robot_info(robot)
                    if self.session.robot_info is None:
                        self._mark_waiting_for_robot(f"Robot '{robot}' is not available yet")
                        self._update_gui_state()
                        return self._snapshot()
                    try:
                        self.session.current_joints = client.get_current_joints(robot)
                        self.session.current_ee_pose = self._pose_from_rpc(
                            client.get_ee_pose(robot)
                        )
                        self.session.manipulation_state = str(client.get_state())
                        self.session.error = str(client.get_error() or "")
                    except TimeoutError as e:
                        self._reset_manipulation_client()
                        self._mark_waiting_for_robot(repr(e))
                        self._update_gui_state()
                        return self._snapshot()
                    except (RuntimeError, ValueError) as e:
                        self._mark_waiting_for_robot(str(e))
                        self._update_gui_state()
                        return self._snapshot()
                    self._ensure_robot_ui(robot)
                else:
                    self.session.backend_status = BackendConnectionStatus.WAITING_FOR_ROBOT
                    self.session.manipulation_state = "NO_ROBOT"
            except Exception as e:
                self.session.backend_status = BackendConnectionStatus.DISCONNECTED
                self.session.manipulation_state = "DISCONNECTED"
                self.session.error = str(e) or traceback.format_exc()
            self._update_gui_state()
            return self._snapshot()

    @rpc
    def get_panel_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot()

    def _mark_waiting_for_robot(self, message: str) -> None:
        self.session.backend_status = BackendConnectionStatus.WAITING_FOR_ROBOT
        self.session.manipulation_state = "WAITING_FOR_ROBOT"
        self.session.error = message

    def _select_robot(self, robot_name: str) -> None:
        self._ensure_controller().select_robot(robot_name)

    def _select_robot_impl(self, robot_name: str) -> None:
        if robot_name in {"", "No robots"}:
            return
        with self._lock:
            if self.session.selected_robot != robot_name:
                self.session.selected_robot = robot_name
                self.session.plan_state.status = PlanStatus.STALE
                self.session.target_status = TargetStatus.EMPTY
                self._joint_sliders.clear()

    def _update_robot_dropdown(self, robots: Sequence[str]) -> None:
        self._ensure_gui().update_robot_dropdown(robots)

    def _ensure_robot_ui(self, robot_name: str) -> None:
        info = self.session.robot_info or {}
        self._ensure_scene_nodes(robot_name, info)
        if not self._joint_sliders:
            self._build_joint_sliders(robot_name, info)
        self._update_preset_options(info)
        self._update_current_robot(robot_name)

    def _ensure_scene_nodes(self, robot_name: str, info: dict[str, Any]) -> None:
        scene = self._ensure_scene()
        prepared_urdf_path = self._prepared_urdf_path
        if getattr(prepared_urdf_path, "__func__", None) is not type(self)._prepared_urdf_path:
            scene.prepared_urdf_path = prepared_urdf_path
        scene.ensure_scene_nodes(robot_name, info, self._target_pose_changed)

    def _build_joint_sliders(self, robot_name: str, info: dict[str, Any]) -> None:
        self._ensure_gui().build_joint_sliders(robot_name, info, self._joint_slider_changed)

    def _update_preset_options(self, info: dict[str, Any]) -> None:
        self._ensure_gui().update_preset_options(info)

    def _update_current_robot(self, robot_name: str) -> None:
        self._ensure_scene().update_current_robot(robot_name)

    def _target_pose_changed(self, target: Any) -> None:
        self._ensure_controller().target_pose_changed(target)

    def _target_pose_changed_impl(self, target: Any) -> None:
        with self._lock:
            if self.session.sync_source == "joints" or self.session.selected_robot is None:
                return
            wxyz = tuple(target.wxyz)
            pose = self._make_pose(tuple(target.position), (wxyz[1], wxyz[2], wxyz[3], wxyz[0]))
            self.session.cartesian_target = pose
            sequence_id = self.session.next_sequence_id()
            self._preview_worker.submit(
                PreviewRequest(sequence_id, "cartesian", self.session.selected_robot, pose=pose)
            )

    def _joint_slider_changed(self, _joint_name: str) -> None:
        self._ensure_controller().joint_slider_changed(_joint_name)

    def _joint_slider_changed_impl(self, _joint_name: str) -> None:
        with self._lock:
            if self.session.sync_source == "cartesian" or self.session.selected_robot is None:
                return
            info = self.session.robot_info or {}
            names = list(info.get("joint_names") or self._joint_sliders.keys())
            joints = [float(self._joint_sliders[name].value) for name in names]
            self.session.joint_target = joints
            sequence_id = self.session.next_sequence_id()
            self._preview_worker.submit(
                PreviewRequest(
                    sequence_id,
                    "joints",
                    self.session.selected_robot,
                    joints=self._make_joint_state(names, joints),
                )
            )

    def _apply_preset(self, preset: str) -> None:
        self._ensure_controller().apply_preset(preset)

    def _apply_preset_impl(self, preset: str) -> None:
        if preset in {"", "Select preset..."} or self.session.selected_robot is None:
            return
        with self._lock:
            info = self.session.robot_info or {}
            if preset == "Current":
                self.session.cartesian_target = self.session.current_ee_pose
                self.session.joint_target = self.session.current_joints
                self._sync_controls_from_targets()
                if self.session.selected_robot and self.session.current_ee_pose:
                    sequence_id = self.session.next_sequence_id()
                    self._preview_worker.submit(
                        PreviewRequest(
                            sequence_id,
                            "cartesian",
                            self.session.selected_robot,
                            pose=self.session.current_ee_pose,
                        )
                    )
            else:
                joints = info.get("init_joints") if preset == "Init" else info.get("home_joints")
                if joints is not None:
                    self.session.joint_target = list(joints)
                    self._sync_controls_from_targets()
                    sequence_id = self.session.next_sequence_id()
                    self._preview_worker.submit(
                        PreviewRequest(
                            sequence_id,
                            "joints",
                            self.session.selected_robot,
                            joints=self._make_joint_state(
                                info.get("joint_names") or [], list(joints)
                            ),
                        )
                    )
            self._handles["preset"].value = "Select preset..."

    def _handle_preview_request(self, request: PreviewRequest) -> dict[str, Any]:
        try:
            if self._client is None:
                return {"success": False, "status": "DISCONNECTED", "message": "Disconnected"}
            client = self._client
            if request.source == "cartesian" and request.pose is not None:
                return self._call_preview_with_timeout(
                    lambda: dict(client.solve_ik_preview(request.pose, request.robot_name)),
                    "IK_TIMEOUT",
                )
            if request.source == "joints" and request.joints is not None:
                return self._call_preview_with_timeout(
                    lambda: dict(client.solve_fk_preview(request.joints, request.robot_name)),
                    "FK_TIMEOUT",
                )
            return {"success": False, "status": "INVALID", "message": "Invalid preview request"}
        except Exception as e:
            return {"success": False, "status": "ERROR", "message": str(e)}

    def _call_preview_with_timeout(self, call: Any, timeout_status: str) -> dict[str, Any]:
        return self._ensure_backend().call_preview_with_timeout(call, timeout_status)

    def _apply_preview_result(self, request: PreviewRequest, result: dict[str, Any]) -> None:
        with self._lock:
            if request.sequence_id != self.session.latest_sequence_id:
                return
            success = bool(result.get("success")) and bool(result.get("collision_free", True))
            if success:
                self.session.feasibility.status = FeasibilityStatus.FEASIBLE
                self.session.feasibility.message = "Feasible"
                self.session.target_status = TargetStatus.FEASIBLE
                joint_state = result.get("joint_state")
                if joint_state is not None:
                    self.session.joint_target = list(joint_state.position)
                pose = self._pose_from_rpc(result.get("pose"))
                if pose is not None:
                    self.session.cartesian_target = pose
                self._sync_controls_from_targets()
            else:
                status = str(result.get("status") or "invalid")
                self.session.feasibility.status = (
                    FeasibilityStatus.COLLISION
                    if status == "COLLISION"
                    else FeasibilityStatus.IK_FAILED
                )
                self.session.feasibility.message = str(result.get("message") or status)
                self.session.target_status = TargetStatus.INFEASIBLE
            self._set_target_visual_state(
                self.session.feasibility.status == FeasibilityStatus.FEASIBLE
            )
            self._update_gui_state()

    def _sync_controls_from_targets(self) -> None:
        self._ensure_gui().sync_controls_from_targets(
            set_ghost_joints=lambda joints: self._ensure_scene().set_selected_ghost_joints(joints)
        )

    def _run_operation(self, name: str, operation: Any) -> None:
        action = {
            "Preview": ActionStatus.PREVIEWING,
            "Execute": ActionStatus.EXECUTING,
            "Cancel": ActionStatus.CANCELLING,
            "Clear Plan": ActionStatus.CLEARING_PLAN,
        }.get(name, ActionStatus.IDLE)

        def run() -> None:
            with self._lock:
                self.session.action_status = action
                self._update_gui_state()
            try:
                operation()
            except Exception as e:
                with self._lock:
                    self.session.error = str(e)
                    self.session.plan_state.status = PlanStatus.FAILED
            finally:
                with self._lock:
                    if self.session.action_status == action:
                        self.session.action_status = ActionStatus.IDLE
                    self._update_gui_state()

        threading.Thread(target=run, daemon=True).start()

    def _plan(self) -> None:
        self._ensure_controller().plan()

    def _plan_impl(self) -> None:
        if (
            self._client is None
            or not self._can_plan_for_operation()
            or self.session.selected_robot is None
        ):
            return
        robot = self.session.selected_robot
        current = self.session.current_joints
        target = self.session.joint_target
        self.session.plan_state.status = PlanStatus.PLANNING
        if target is not None:
            names = list((self.session.robot_info or {}).get("joint_names") or [])
            ok = bool(self._client.plan_to_joints(self._make_joint_state(names, target), robot))
        elif self.session.cartesian_target is not None:
            ok = bool(self._client.plan_to_pose(self.session.cartesian_target, robot))
        else:
            ok = False
        if ok:
            path = self._client.get_planned_path(robot)
            self.session.plan_state.status = PlanStatus.FRESH
            self.session.plan_state.robot = robot
            self.session.plan_state.target_pose = self.session.cartesian_target
            self.session.plan_state.target_joints = target
            self.session.plan_state.start_joints_snapshot = (
                list(current) if current is not None else None
            )
            self.session.plan_state.planned_path = list(path or [])
            self._render_plan_path(self.session.plan_state.planned_path)
        else:
            self.session.plan_state.status = PlanStatus.FAILED
            self.session.error = str(self._client.get_error() or "Planning failed")

    def _preview(self) -> None:
        self._ensure_controller().preview()

    def _preview_impl(self) -> None:
        if self._client is None or self.session.selected_robot is None:
            return
        path = list(self._client.get_planned_path(self.session.selected_robot) or [])
        self._render_plan_path(path)
        if path:
            self.session.error = "Previewing planned path in Viser"
            self._animate_ghost_path(path, self.panel_config.preview_duration)
        else:
            self.session.error = "No planned path to preview"
            self.session.plan_state.status = PlanStatus.FAILED

    def _execute(self) -> None:
        self._ensure_controller().execute()

    def _execute_impl(self) -> None:
        if (
            self._client is None
            or self.session.selected_robot is None
            or not self._can_execute_for_operation()
        ):
            self.session.error = "Execute is not available for the current plan"
            return
        self.session.plan_state.status = PlanStatus.EXECUTING
        if bool(self._client.execute(self.session.selected_robot)):
            self.session.error = "Execution accepted"
        else:
            self.session.plan_state.status = PlanStatus.FAILED
            self.session.error = str(self._client.get_error() or "Execution failed")

    def _cancel(self) -> None:
        self._ensure_controller().cancel()

    def _cancel_impl(self) -> None:
        if self._client is not None:
            self._client.cancel()

    def _clear_plan(self) -> None:
        self._ensure_controller().clear_plan()

    def _clear_plan_impl(self) -> None:
        if self._client is not None:
            self._client.clear_planned_path()
        self.session.plan_state = self.session.plan_state.__class__()
        self._render_plan_path([])

    def _render_plan_path(self, path: Sequence[JointState]) -> None:
        self._ensure_scene().render_plan_path(path)

    def _animate_ghost_path(self, path: Sequence[JointState], duration: float) -> None:
        self._ensure_scene().animate_ghost_path(path, duration, self.panel_config.preview_fps)

    def _interpolate_joint_path(
        self, path: Sequence[JointState], duration: float, fps: float
    ) -> list[list[float]]:
        return interpolate_joint_path(path, duration, fps)

    def _update_gui_state(self) -> None:
        self._ensure_gui().update_gui_state(
            can_execute=self._can_execute_from_ui(),
            set_target_visual_state=self._set_target_visual_state,
        )

    def _snapshot(self) -> dict[str, Any]:
        return {
            "connected": self.session.connected,
            "selected_robot": self.session.selected_robot,
            "module_state": self.session.module_state,
            "runtime": self.session.runtime.value,
            "backend_status": self.session.backend_status.value,
            "target_status": self.session.target_status.value,
            "action_status": self.session.action_status.value,
            "error": self.session.error,
            "feasibility": self.session.feasibility.status.value,
            "plan_status": self.session.plan_state.status.value,
            "can_plan": self.session.can_plan(),
            "can_execute": self._can_execute_from_ui(),
        }

    def _pose_from_rpc(self, pose: Any) -> Pose | None:
        if pose is None:
            return None
        if isinstance(pose, Pose):
            return pose
        return self._make_pose(pose.position, pose.orientation)

    def _make_pose(self, position: Any, orientation: Any) -> Pose:
        pose = Pose.__new__(Pose)
        pose.position = Vector3(position)
        pose.orientation = Quaternion(orientation)
        return pose

    def _make_joint_state(self, names: Sequence[str], positions: Sequence[float]) -> JointState:
        joint_state = JointState.__new__(JointState)
        joint_state.ts = time.time()
        joint_state.frame_id = ""
        joint_state.name = list(names)
        joint_state.position = list(positions)
        joint_state.velocity = []
        joint_state.effort = []
        return joint_state

    def _prepared_urdf_path(self, info: dict[str, Any]) -> Any:
        return self._ensure_scene().prepared_urdf_path(info)

    def _set_urdf_joints(self, urdf: Any, joints: Sequence[float]) -> None:
        self._ensure_scene().set_urdf_joints(urdf, joints)

    def _viser_joint_configuration(
        self, urdf: Any, joint_names: Sequence[str], joints: Sequence[float]
    ) -> list[float]:
        return self._ensure_scene().viser_joint_configuration(urdf, joint_names, joints)

    def _viser_actuated_joint_names(self, urdf: Any) -> tuple[str, ...]:
        return self._ensure_scene().viser_actuated_joint_names(urdf)

    def _set_target_visual_state(self, feasible: bool) -> None:
        self._ensure_scene().set_target_visual_state(feasible)

    def _set_urdf_mesh_material(
        self, urdf: Any | None, color: tuple[int, int, int], opacity: float
    ) -> None:
        self._ensure_scene().set_urdf_mesh_material(urdf, color, opacity)

    def _can_execute_from_ui(self, require_no_operation: bool = True) -> bool:
        if not self.panel_config.allow_plan_execute:
            return False
        if require_no_operation and not self.session.can_execute(
            self.panel_config.current_match_tolerance
        ):
            return False
        if not require_no_operation and not self._can_execute_for_operation():
            return False
        return True

    def _can_plan_for_operation(self) -> bool:
        return (
            self.session.runtime == PanelRuntime.RUNNING
            and self.session.backend_status == BackendConnectionStatus.READY
            and self.session.selected_robot is not None
            and self.session.target_status == TargetStatus.FEASIBLE
            and self.session.manipulation_state in {"IDLE", "COMPLETED"}
        )

    def _can_execute_for_operation(self) -> bool:
        previous_action = self.session.action_status
        try:
            self.session.action_status = ActionStatus.IDLE
            return self.session.can_execute(self.panel_config.current_match_tolerance)
        finally:
            self.session.action_status = previous_action


ViserManipulationPanelModule.__annotations__["config"] = ViserManipulationPanelConfig
viser_manipulation_panel = ViserManipulationPanelModule.blueprint
