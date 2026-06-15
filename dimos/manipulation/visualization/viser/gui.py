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

from typing import Protocol, SupportsFloat, cast

from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    FeasibilityStatus,
    OperationWorker,
    PanelPlanState,
    PanelRuntime,
    PanelState,
    PlanStatus,
    TargetEvaluationRequest,
    TargetEvaluationWorker,
    TargetStatus,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState


class _GuiServer(Protocol):
    gui: object


class _ValueHandle(Protocol):
    value: object


class _DisabledHandle(Protocol):
    disabled: bool


class _OptionHandle(Protocol):
    options: list[str]
    values: list[str]


class _SliderHandle(Protocol):
    value: float


class _RemovableHandle(Protocol):
    def remove(self) -> None: ...


class _UpdateHandle(Protocol):
    def on_update(self, callback: object) -> object: ...


class _ClickHandle(Protocol):
    def on_click(self, callback: object) -> object: ...


class _FolderHandle(Protocol):
    def __enter__(self) -> object: ...
    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> object: ...


class _FolderGui(Protocol):
    def add_folder(self, label: str, *, expand_by_default: bool = False) -> object: ...


class _MarkdownGui(Protocol):
    def add_markdown(self, content: str) -> object: ...


class _TextGui(Protocol):
    def add_text(self, label: str, *, initial_value: str) -> object: ...


class _DropdownGui(Protocol):
    def add_dropdown(self, label: str, *, options: list[str], initial_value: str) -> object: ...


class _ButtonGui(Protocol):
    def add_button(self, label: str, *, disabled: bool = False) -> object: ...


class _CheckboxGui(Protocol):
    def add_checkbox(self, label: str, *, initial_value: bool) -> object: ...


class _SliderGui(Protocol):
    def add_slider(
        self,
        label: str,
        *,
        min: float,
        max: float,
        step: float,
        initial_value: float,
    ) -> object: ...


class _PositionLike(Protocol):
    x: float
    y: float
    z: float


class _QuaternionLike(Protocol):
    w: float
    x: float
    y: float
    z: float


class _Indexable(Protocol):
    def __getitem__(self, index: int) -> object: ...


class _TransformTarget(Protocol):
    position: object
    wxyz: object


class ViserPanelGui:
    """Optional operator panel with parity for the original cc/viser-vis panel."""

    def __init__(
        self,
        server: object,
        adapter: InProcessViserAdapter,
        config: ViserVisualizationConfig,
        scene: ViserManipulationScene | None = None,
    ) -> None:
        self.server = server
        self.adapter = adapter
        self.config = config
        self.scene = scene
        self.state = PanelState(runtime=PanelRuntime.STARTING)
        self._closed = False
        self._suppress_target_callbacks = False
        self._handles: dict[str, object] = {}
        self._joint_sliders: dict[str, _SliderHandle] = {}
        self._worker = TargetEvaluationWorker(
            self._handle_target_evaluation_request,
            self._apply_target_evaluation_result,
        )
        self._operation_worker = OperationWorker(self._set_error)

    def start(self) -> None:
        self._worker.start()
        self._operation_worker.start()
        try:
            self.state.runtime = PanelRuntime.RUNNING
            self._build()
            self.refresh()
        except Exception:
            self.close()
            self.state.runtime = PanelRuntime.FAILED
            raise

    def close(self) -> None:
        self._closed = True
        self.state.runtime = PanelRuntime.STOPPING
        self._worker.stop()
        self._operation_worker.stop(timeout=None)
        self._clear_joint_sliders()
        self._handles.clear()
        self.state.runtime = PanelRuntime.STOPPED

    def refresh(self) -> None:
        robots = self.adapter.list_robots()
        self.state.backend_status = (
            BackendConnectionStatus.READY if robots else BackendConnectionStatus.WAITING_FOR_ROBOT
        )
        if self.state.selected_robot is None and robots:
            self.state.selected_robot = robots[0]
            self.state.target_status = TargetStatus.EMPTY
            self._build_joint_sliders()
        self._sync_robot_dropdown(robots)
        self._refresh_selected_robot_state()
        self._ensure_scene_controls()
        self._sync_preset_dropdown()
        self._update_status_text()
        self._update_control_state()

    def _gui(self) -> object | None:
        try:
            return cast("_GuiServer", self.server).gui
        except AttributeError:
            return None

    def _build(self) -> None:
        gui = self._gui()
        if gui is None:
            return
        folder = None
        if hasattr(gui, "add_folder"):
            try:
                folder = cast("_FolderGui", gui).add_folder(
                    "Manipulation Panel", expand_by_default=True
                )
            except TypeError:
                folder = cast("_FolderGui", gui).add_folder("Manipulation Panel")
            self._handles["panel_folder"] = folder
        if folder is not None and hasattr(folder, "__enter__"):
            with cast("_FolderHandle", folder):
                self._build_panel_controls(gui)
        else:
            self._build_panel_controls(gui)

    def _build_panel_controls(self, gui: object) -> None:
        if hasattr(gui, "add_markdown"):
            self._handles["status"] = cast("_MarkdownGui", gui).add_markdown(
                "Starting manipulation panel..."
            )
        elif hasattr(gui, "add_text"):
            self._handles["status"] = cast("_TextGui", gui).add_text(
                "Status", initial_value="Starting"
            )
        robots = self.adapter.list_robots()
        self._build_scene_controls(gui)
        if hasattr(gui, "add_dropdown"):
            self._handles["robot"] = cast("_DropdownGui", gui).add_dropdown(
                "Robot",
                options=robots or [""],
                initial_value=robots[0] if robots else "",
            )
            cast("_UpdateHandle", self._handles["robot"]).on_update(
                lambda event: self._select_robot(event.target.value)
            )
            self._handles["preset"] = cast("_DropdownGui", gui).add_dropdown(
                "Target Preset",
                options=["Select preset...", "Current"],
                initial_value="Select preset...",
            )
            cast("_UpdateHandle", self._handles["preset"]).on_update(
                lambda event: self._apply_preset(event.target.value)
            )
        if hasattr(gui, "add_button"):
            self._handles["plan"] = self._add_button(gui, "Plan", disabled=True)
            cast("_ClickHandle", self._handles["plan"]).on_click(lambda _: self._submit_plan())
            self._handles["preview"] = self._add_button(gui, "Preview", disabled=True)
            cast("_ClickHandle", self._handles["preview"]).on_click(
                lambda _: self._submit_preview()
            )
            self._handles["execute"] = self._add_button(gui, "Execute", disabled=True)
            cast("_ClickHandle", self._handles["execute"]).on_click(
                lambda _: self._submit_execute()
            )
            self._handles["cancel"] = self._add_button(gui, "Cancel")
            cast("_ClickHandle", self._handles["cancel"]).on_click(lambda _: self._submit_cancel())
            self._handles["clear"] = self._add_button(gui, "Clear plan")
            cast("_ClickHandle", self._handles["clear"]).on_click(lambda _: self._submit_clear())
        self._build_joint_sliders()

    def _build_scene_controls(self, gui: object) -> None:
        if (
            self.scene is None
            or not hasattr(gui, "add_checkbox")
            or not hasattr(self.scene, "has_reference_grid")
        ):
            return
        if not self.scene.has_reference_grid():
            return
        handle = cast("_CheckboxGui", gui).add_checkbox("Scene grid", initial_value=True)
        self._handles["scene_grid"] = handle
        if hasattr(handle, "on_update"):
            cast("_UpdateHandle", handle).on_update(
                lambda event: self._set_scene_grid_visible(event.target.value)
            )

    def _set_scene_grid_visible(self, visible: object) -> None:
        if self.scene is None:
            return
        self.scene.set_reference_grid_visible(bool(visible))

    def _add_button(self, gui: object, label: str, *, disabled: bool = False) -> object:
        try:
            handle = cast("_ButtonGui", gui).add_button(label, disabled=disabled)
        except TypeError:
            handle = cast("_ButtonGui", gui).add_button(label)
            try:
                cast("_DisabledHandle", handle).disabled = disabled
            except Exception:
                pass
        return handle

    def _refresh_selected_robot_state(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            self.state.robot_info = None
            self.state.current_joints = None
            self.state.current_ee_pose = None
            self.state.manipulation_state = self.adapter.get_module_state()
            return
        self.state.robot_info = self.adapter.get_robot_info(robot_name)
        current = self.adapter.get_current_joint_state(robot_name)
        self.state.current_joints = list(current.position) if current is not None else None
        self.state.current_ee_pose = self.adapter.get_ee_pose(robot_name)
        self.state.manipulation_state = self.adapter.get_module_state()
        adapter_error = self.adapter.get_error()
        if adapter_error:
            self.state.error = adapter_error

    def _ensure_scene_controls(self) -> None:
        if (
            self.scene is None
            or self.state.selected_robot is None
            or not hasattr(self.scene, "ensure_target_controls")
        ):
            return
        robot_id = self.adapter.robot_id_for_name(self.state.selected_robot)
        if robot_id is None:
            return
        self._handles["ee_control"] = self.scene.ensure_target_controls(
            str(robot_id), self._on_transform_update
        )
        if (
            self.state.target_status == TargetStatus.EMPTY
            and self.state.current_ee_pose is not None
        ):
            self.state.cartesian_target = self.state.current_ee_pose
            self._suppress_target_callbacks = True
            try:
                self.scene.set_target_pose(str(robot_id), self.state.current_ee_pose)
            finally:
                self._suppress_target_callbacks = False

    def _build_joint_sliders(self) -> None:
        gui = self._gui()
        if gui is None or not hasattr(gui, "add_slider") or self.state.selected_robot is None:
            return
        config = self.adapter.get_robot_config(self.state.selected_robot)
        if config is None:
            return
        current = self.adapter.get_current_joint_state(self.state.selected_robot)
        values = list(current.position) if current is not None else [0.0] * len(config.joint_names)
        self._clear_joint_sliders()
        try:
            joint_limits_lower = config.joint_limits_lower
        except AttributeError:
            joint_limits_lower = None
        try:
            joint_limits_upper = config.joint_limits_upper
        except AttributeError:
            joint_limits_upper = None
        for index, joint_name in enumerate(config.joint_names):
            lower, upper = (-3.14, 3.14)
            if joint_limits_lower is not None and index < len(joint_limits_lower):
                lower = joint_limits_lower[index]
            if joint_limits_upper is not None and index < len(joint_limits_upper):
                upper = joint_limits_upper[index]
            handle = cast("_SliderGui", gui).add_slider(
                joint_name,
                min=float(lower),
                max=float(upper),
                step=0.001,
                initial_value=float(values[index] if index < len(values) else 0.0),
            )
            if hasattr(handle, "on_update"):
                cast("_UpdateHandle", handle).on_update(
                    lambda _event, name=joint_name: self._on_joint_slider_update(name)
                )
            self._joint_sliders[joint_name] = cast("_SliderHandle", handle)

    def _clear_joint_sliders(self) -> None:
        for handle in self._joint_sliders.values():
            try:
                cast("_RemovableHandle", handle).remove()
            except AttributeError:
                pass
        self._joint_sliders.clear()

    def _select_robot(self, robot_name: str) -> None:
        if (robot_name or None) == self.state.selected_robot:
            self.refresh()
            return
        self.state.selected_robot = robot_name or None
        self.state.target_status = TargetStatus.EMPTY
        self.state.feasibility.status = FeasibilityStatus.UNKNOWN
        self.state.plan_state = PanelPlanState()
        self._build_joint_sliders()
        self._sync_preset_dropdown()
        self.refresh()

    def _sync_robot_dropdown(self, robots: list[str]) -> None:
        handle = self._handles.get("robot")
        if handle is None:
            return
        options = robots or [""]
        for attr in ("options", "values"):
            if hasattr(handle, attr):
                try:
                    if attr == "options":
                        cast("_OptionHandle", handle).options = options
                    else:
                        cast("_OptionHandle", handle).values = options
                except Exception:
                    pass
        if hasattr(handle, "value") and self.state.selected_robot in robots:
            try:
                cast("_ValueHandle", handle).value = self.state.selected_robot
            except Exception:
                pass

    def _sync_preset_dropdown(self) -> None:
        handle = self._handles.get("preset")
        if handle is None or self.state.selected_robot is None:
            return
        info = self.adapter.get_robot_info(self.state.selected_robot) or {}
        config = self.adapter.get_robot_config(self.state.selected_robot)
        options = ["Select preset..."]
        if (
            info.get("init_joints") is not None
            or self.adapter.get_init_joints(self.state.selected_robot) is not None
        ):
            options.append("Init")
        options.append("Current")
        home_joints = None
        if config is not None:
            try:
                home_joints = config.home_joints
            except AttributeError:
                home_joints = None
        if info.get("home_joints") is not None or home_joints is not None:
            options.append("Home")
        for attr in ("options", "values"):
            if hasattr(handle, attr):
                try:
                    if attr == "options":
                        cast("_OptionHandle", handle).options = options
                    else:
                        cast("_OptionHandle", handle).values = options
                except Exception:
                    pass

    def _apply_preset(self, preset: str) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        config = self.adapter.get_robot_config(robot_name)
        if config is None:
            return
        if preset == "Current":
            current = self.adapter.get_current_joint_state(robot_name)
            values = list(current.position) if current is not None else []
        elif preset == "Init":
            init = self.adapter.get_init_joints(robot_name)
            values = list(init.position) if init is not None else []
        elif preset == "Home":
            try:
                values = list(config.home_joints or [])
            except AttributeError:
                values = []
        else:
            return
        self._set_slider_values(config.joint_names, values)
        self.state.joint_target = [float(value) for value in values]
        self._submit_joint_target_evaluation()
        self.refresh()

    def _set_slider_values(self, joint_names: list[str], values: list[float]) -> None:
        self._suppress_target_callbacks = True
        try:
            for joint_name, value in zip(joint_names, values, strict=False):
                handle = self._joint_sliders.get(joint_name)
                if handle is not None:
                    handle.value = float(value)
        finally:
            self._suppress_target_callbacks = False

    def _target_from_sliders(self, robot_name: str) -> JointState | None:
        config = self.adapter.get_robot_config(robot_name)
        if config is None:
            self._set_error("No robot config")
            return None
        values = [
            float(self._joint_sliders[name].value)
            for name in config.joint_names
            if name in self._joint_sliders
        ]
        return self.adapter.joints_from_values(config.joint_names, values)

    def _on_joint_slider_update(self, _joint_name: str) -> None:
        if self._suppress_target_callbacks:
            return
        self._submit_joint_target_evaluation()

    def _on_transform_update(self, target: object) -> None:
        if self._suppress_target_callbacks or self.state.selected_robot is None:
            return
        pose = self._pose_from_transform_target(target)
        if pose is None:
            return
        self.state.cartesian_target = pose
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="cartesian",
                robot_name=self.state.selected_robot,
                pose=pose,
            )
        )
        self.refresh()

    def _submit_joint_target_evaluation(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        target = self._target_from_sliders(robot_name)
        if target is None:
            return
        self.state.joint_target = list(target.position)
        self._move_joint_target_visuals(robot_name, target)
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="joints",
                robot_name=robot_name,
                joints=target,
            )
        )
        self.refresh()

    def _move_joint_target_visuals(self, robot_name: str, target: JointState) -> None:
        """Optimistically move target visuals before collision/feasibility returns."""
        config = self.adapter.get_robot_config(robot_name)
        robot_id = self.adapter.robot_id_for_name(robot_name)
        if self.scene is not None and config is not None and robot_id is not None:
            self.scene.set_target_joints(str(robot_id), config.joint_names, list(target.position))
            pose = self.adapter.get_ee_pose(robot_name, target)
            if pose is not None:
                self._suppress_target_callbacks = True
                try:
                    self.scene.set_target_pose(str(robot_id), pose)
                finally:
                    self._suppress_target_callbacks = False

    def _handle_target_evaluation_request(
        self, request: TargetEvaluationRequest
    ) -> dict[str, object]:
        if request.source == "cartesian":
            if request.pose is None:
                return {"success": False, "status": "INVALID", "message": "No pose target"}
            return self.adapter.evaluate_pose_target(request.pose, request.robot_name)
        if request.joints is None:
            return {"success": False, "status": "INVALID", "message": "No joint target"}
        return self.adapter.evaluate_joint_target(request.joints, request.robot_name)

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: dict[str, object]
    ) -> None:
        if request.sequence_id != self.state.latest_sequence_id:
            return
        collision_free = bool(result.get("collision_free", False))
        success = bool(result.get("success", False))
        self.state.feasibility.status = self._feasibility_status(result, success, collision_free)
        self.state.feasibility.message = str(result.get("message", ""))
        self.state.target_status = (
            TargetStatus.FEASIBLE if success and collision_free else TargetStatus.INFEASIBLE
        )
        self.state.error = "" if success and collision_free else self.state.feasibility.message
        if request.source == "joints":
            joint_state = result.get("joint_state")
            if isinstance(joint_state, JointState):
                self.state.joint_target = list(joint_state.position)
        if request.source == "cartesian":
            joint_state = result.get("joint_state")
            if isinstance(joint_state, JointState):
                self.state.joint_target = list(joint_state.position)
            pose = result.get("pose") or result.get("ee_pose")
            if isinstance(pose, Pose):
                self.state.cartesian_target = pose
            self._sync_controls_from_targets()
        self._update_target_visual_state()
        self.refresh()

    def _sync_controls_from_targets(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        config = self.adapter.get_robot_config(robot_name)
        if config is not None and self.state.joint_target is not None:
            self._set_slider_values(list(config.joint_names), list(self.state.joint_target))
            robot_id = self.adapter.robot_id_for_name(robot_name)
            if self.scene is not None and robot_id is not None:
                self.scene.set_target_joints(
                    str(robot_id), config.joint_names, self.state.joint_target
                )
        if self.scene is not None and self.state.cartesian_target is not None:
            robot_id = self.adapter.robot_id_for_name(robot_name)
            if robot_id is not None:
                self._suppress_target_callbacks = True
                try:
                    self.scene.set_target_pose(str(robot_id), self.state.cartesian_target)
                finally:
                    self._suppress_target_callbacks = False

    def _update_status_text(self) -> None:
        current = self.state.current_joints
        status = [
            "### Manipulation Panel",
            f"Robot: `{self.state.selected_robot or 'none'}`",
            f"Module: `{self.state.module_state}`",
            f"Backend: `{self.state.backend_status.value}`",
            f"Target: `{self.state.target_status.value}`",
            f"Feasibility: `{self.state.feasibility.status.value}`",
            f"Plan: `{self.state.plan_state.status.value}`",
            f"Action: `{self.state.action_status.value}`",
        ]
        if self.state.selected_robot is not None:
            status.append(
                f"State stale: `{self.adapter.is_state_stale(self.state.selected_robot)}`"
            )
        if current is not None:
            status.append(f"Current joints: `{[round(v, 3) for v in current]}`")
        if self.state.last_result:
            status.append(f"Last result: `{self.state.last_result}`")
        if self.state.error:
            status.append(f"Error: `{self.state.error}`")
        self._set_handle_value("status", "\n\n".join(status))

    def _update_control_state(self) -> None:
        self._set_disabled("plan", not self.state.can_plan())
        self._set_disabled("preview", not self.state.can_preview())
        self._set_disabled(
            "execute",
            not (
                self.config.allow_plan_execute
                and self.state.can_execute(self.config.current_match_tolerance)
            ),
        )
        self._set_disabled("cancel", not self.state.can_cancel())
        self._update_target_visual_state()

    def _update_target_visual_state(self) -> None:
        if (
            self.scene is None
            or self.state.selected_robot is None
            or not hasattr(self.scene, "set_target_visual_state")
        ):
            return
        robot_id = self.adapter.robot_id_for_name(self.state.selected_robot)
        if robot_id is None:
            return
        self.scene.set_target_visual_state(
            str(robot_id), self.state.feasibility.status == FeasibilityStatus.FEASIBLE
        )

    def _submit_plan_to_sliders(self) -> None:
        self._submit_plan()

    def _submit_plan(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        if not self.state.can_plan():
            self._set_error("Cannot plan until target is feasible and manipulation is idle")
            return

        def operation() -> None:
            self.state.action_status = ActionStatus.RUNNING
            self.state.plan_state.status = PlanStatus.PLANNING
            target = self._target_from_sliders(robot_name)
            if target is None:
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation("plan_to_joints=False", clear_error=False)
                return
            ok = self.adapter.plan_to_joints(target, robot_name)
            if ok:
                path = self.adapter.get_planned_path(robot_name)
                self.state.plan_state.status = PlanStatus.FRESH
                self.state.plan_state.robot = robot_name
                self.state.plan_state.target_joints = list(target.position)
                self.state.plan_state.target_pose = self.state.cartesian_target
                self.state.plan_state.start_joints_snapshot = list(self.state.current_joints or [])
                self.state.plan_state.planned_path = path
            else:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"plan_to_joints={ok}")

        self._operation_worker.submit(operation)

    def _submit_preview(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        if not self.state.can_preview():
            self._set_error("No fresh plan to preview")
            return

        def operation() -> None:
            self.state.action_status = ActionStatus.PREVIEWING
            ok = self.adapter.preview_path(robot_name)
            self._finish_operation(f"preview={ok}")

        self._operation_worker.submit(operation)

    def _submit_execute(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        if not self.config.allow_plan_execute:
            self._set_error("Panel execution disabled; set allow_plan_execute=True to enable")
            return
        if not self.state.can_execute(self.config.current_match_tolerance):
            self._set_error(
                "Cannot execute: require feasible fresh plan and matching current joints"
            )
            return

        def operation() -> None:
            self.state.action_status = ActionStatus.EXECUTING
            self.state.plan_state.status = PlanStatus.EXECUTING
            ok = self.adapter.execute(robot_name)
            if not ok:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"execute={ok}")

        self._operation_worker.submit(operation)

    def _submit_cancel(self) -> None:
        def operation() -> None:
            self.state.action_status = ActionStatus.CANCELLING
            ok = self.adapter.cancel()
            self.state.action_status = ActionStatus.IDLE
            self._finish_operation(f"cancel={ok}")

        self._operation_worker.submit(operation)

    def _submit_clear(self) -> None:
        def operation() -> None:
            self.state.action_status = ActionStatus.CLEARING_PLAN
            ok = self.adapter.clear_planned_path()
            self.state.plan_state = PanelPlanState()
            self._finish_operation(f"clear={ok}")

        self._operation_worker.submit(operation)

    def _finish_operation(self, result: str, *, clear_error: bool = True) -> None:
        if self._closed:
            return
        self.state.action_status = ActionStatus.IDLE
        if clear_error:
            self.state.error = ""
        self.state.last_result = result
        self.refresh()

    def _set_error(self, message: str) -> None:
        if self._closed:
            return
        self.state.action_status = ActionStatus.FAILED
        self.state.error = message
        self.refresh()

    def _set_handle_value(self, key: str, value: str) -> None:
        handle = self._handles.get(key)
        if handle is not None and hasattr(handle, "value"):
            cast("_ValueHandle", handle).value = value

    def _set_disabled(self, key: str, disabled: bool) -> None:
        handle = self._handles.get(key)
        if handle is not None and hasattr(handle, "disabled"):
            cast("_DisabledHandle", handle).disabled = disabled

    def _pose_from_transform_target(self, target: object) -> Pose | None:
        try:
            transform = cast("_TransformTarget", target)
            position = transform.position
        except AttributeError:
            return None
        try:
            wxyz = transform.wxyz
        except AttributeError:
            wxyz = None
        xyz = self._xyz_from_value(position)
        if xyz is None:
            return None
        px, py, pz = xyz
        if wxyz is None:
            return Pose({"position": [px, py, pz], "orientation": [0.0, 0.0, 0.0, 1.0]})
        quaternion = self._wxyz_from_value(wxyz)
        if quaternion is None:
            return None
        qw, qx, qy, qz = quaternion
        return Pose({"position": [px, py, pz], "orientation": [qx, qy, qz, qw]})

    def _xyz_from_value(self, value: object) -> tuple[float, float, float] | None:
        try:
            point = cast("_PositionLike", value)
            return float(point.x), float(point.y), float(point.z)
        except (AttributeError, TypeError, ValueError):
            pass
        try:
            sequence = cast("_Indexable", value)
            return (
                self._to_float(sequence[0]),
                self._to_float(sequence[1]),
                self._to_float(sequence[2]),
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return None

    def _wxyz_from_value(self, value: object) -> tuple[float, float, float, float] | None:
        try:
            quaternion = cast("_QuaternionLike", value)
            return (
                float(quaternion.w),
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
            )
        except (AttributeError, TypeError, ValueError):
            pass
        try:
            sequence = cast("_Indexable", value)
            return (
                self._to_float(sequence[0]),
                self._to_float(sequence[1]),
                self._to_float(sequence[2]),
                self._to_float(sequence[3]),
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return None

    def _to_float(self, value: object) -> float:
        if isinstance(value, str | int | float):
            return float(value)
        return float(cast("SupportsFloat", value))

    def _feasibility_status(
        self, result: dict[str, object], success: bool, collision_free: bool
    ) -> FeasibilityStatus:
        status = str(result.get("status", "")).upper()
        if success and collision_free:
            return FeasibilityStatus.FEASIBLE
        if "COLLISION" in status:
            return FeasibilityStatus.COLLISION
        if "IK" in status:
            return FeasibilityStatus.IK_FAILED
        return FeasibilityStatus.INVALID
