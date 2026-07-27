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

from collections.abc import Mapping, MutableMapping, Sequence
from typing import TypeAlias, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID, PlanningSceneInfo, RobotName
from dimos.manipulation.visualization.operator import (
    JointTargetRequest,
    ManipulationOperator,
    PoseTargetRequest,
    TargetEvaluationResult,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.runtime import VISER_INSTALL_HINT
from dimos.manipulation.visualization.viser.scene import (
    RobotDisplayMode,
    ViserManipulationScene,
)
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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    from viser import (
        GuiApi,
        GuiButtonHandle,
        GuiCheckboxHandle,
        GuiDropdownHandle,
        GuiFolderHandle,
        GuiMarkdownHandle,
        GuiSliderHandle,
        TransformControlsHandle,
        ViserServer,
    )
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

PanelHandle: TypeAlias = (
    GuiFolderHandle
    | GuiMarkdownHandle
    | GuiDropdownHandle[str]
    | GuiButtonHandle
    | GuiCheckboxHandle
    | TransformControlsHandle
)

# Fallback joint-slider range (radians) when a robot config omits joint limits.
DEFAULT_JOINT_LIMITS = (-3.14, 3.14)
PRIMARY_ACTION_COLOR = (0, 102, 179)
ACTIVE_GROUP_COLOR = PRIMARY_ACTION_COLOR
INACTIVE_GROUP_COLOR = (52, 52, 52)


def group_display_name(group: PlanningGroup) -> str:
    return (
        str(group.robot_name)
        if str(group.group_name) == "manipulator"
        else f"{group.robot_name} {group.group_name}"
    )


def _copy_joint_state(state: JointState | None) -> JointState | None:
    return None if state is None else JointState(state)


ROBOT_DISPLAY_LABELS = tuple(mode.value.title() for mode in RobotDisplayMode)
ROBOT_DISPLAY_MODES: dict[str, RobotDisplayMode] = {mode.value: mode for mode in RobotDisplayMode}
ROBOT_DISPLAY_COLLISION_WARNING = (
    "**Collision meshes unavailable.** Showing visual geometry with collision styling."
)


class ViserPanelGui:
    """Viser operator panel for manipulation target editing and plan control."""

    def __init__(
        self,
        server: ViserServer,
        scene_info: PlanningSceneInfo,
        operator: ManipulationOperator | object,
        current_states: MutableMapping[str, JointState],
        config: ViserVisualizationConfig,
        scene: ViserManipulationScene | None = None,
    ) -> None:
        self.server = server
        self.scene_info = scene_info
        self.operator = cast("ManipulationOperator", operator)
        self.current_states = current_states
        self._robots_by_name = {
            config.name: (robot_id, config) for robot_id, config in scene_info.robots.items()
        }
        self._scene_groups_by_id = {group.id: group for group in scene_info.planning_groups}
        self.config = config
        self.scene = scene
        self.state = PanelState(runtime=PanelRuntime.STARTING)
        self._closed = False
        self._operation_sequence_id = 0
        self._suppress_target_callbacks = False
        self._default_group_initialized = False
        self._handles: dict[str, PanelHandle] = {}
        self._joint_sliders: dict[tuple[PlanningGroupID, str], GuiSliderHandle[float]] = {}
        self._worker = TargetEvaluationWorker(
            self._handle_target_evaluation_request,
            self._apply_target_evaluation_result,
        )
        self._operation_worker = OperationWorker(self._set_error)

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("Cannot restart a closed ViserPanelGui")
        if self.state.runtime == PanelRuntime.RUNNING:
            return
        try:
            self._worker.start()
            self._operation_worker.start()
            self.state.runtime = PanelRuntime.RUNNING
            self._build()
            self.refresh()
        except Exception:
            self.close()
            self.state.runtime = PanelRuntime.FAILED
            raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.state.runtime = PanelRuntime.STOPPING
        if self.scene is not None:
            self.scene.cancel_preview_animation()
        self._worker.stop()
        self._operation_worker.stop(timeout=2.0)
        self._clear_joint_sliders()
        self._remove_panel_handles()
        self._handles.clear()
        self.state.runtime = PanelRuntime.STOPPED

    def list_robots(self) -> list[RobotName]:
        return [config.name for config in self.scene_info.robots.values()]

    def list_planning_groups(self) -> list[PlanningGroup]:
        return list(self.scene_info.planning_groups)

    def robot_items(self) -> list[tuple[RobotName, str, RobotModelConfig]]:
        return [
            (config.name, str(robot_id), config)
            for robot_id, config in self.scene_info.robots.items()
        ]

    def robot_id_for_name(self, robot_name: RobotName) -> str | None:
        item = self._robots_by_name.get(robot_name)
        return None if item is None else str(item[0])

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        item = self._robots_by_name.get(robot_name)
        return None if item is None else item[1]

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        init = self.operator.get_init_joints(robot_name)
        if init is None:
            return None
        config = self.get_robot_config(robot_name)
        if config is None:
            return JointState(init)
        values = self._local_values_for_robot(robot_name, init)
        if any(name not in values for name in config.joint_names):
            return JointState(init)
        return JointState(
            {
                "name": list(config.joint_names),
                "position": [values[name] for name in config.joint_names],
            }
        )

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        return None if robot_id is None else _copy_joint_state(self.current_states.get(robot_id))

    def get_group_ee_pose(self, group_id: PlanningGroupID) -> PoseStamped | None:
        group = self._scene_groups_by_id.get(group_id)
        if group is None:
            return None
        targets = self._current_target_for_group(group)
        if group_id not in targets:
            return None
        return self.evaluate_joint_target_set((group_id,), targets).group_poses.get(group_id)

    def _current_target_for_group(self, group: PlanningGroup) -> dict[PlanningGroupID, JointState]:
        current = self.get_current_joint_state(group.robot_name)
        if current is None or len(current.name) != len(current.position):
            return {}
        values = self._local_values_for_robot(group.robot_name, current)
        if any(name not in values for name in group.local_joint_names):
            return {}
        return {
            group.id: JointState(
                {
                    "name": list(group.joint_names),
                    "position": [values[name] for name in group.local_joint_names],
                }
            )
        }

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        return self.get_current_joint_state(robot_name) is None

    def get_module_state(self) -> str:
        return self.operator.status().state

    def get_error(self) -> str:
        return self.operator.status().error

    def reset(self) -> bool:
        return self.operator.reset()

    def evaluate_joint_target_set(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> TargetEvaluationResult:
        names: list[str] = []
        positions: list[float] = []
        for group_id in group_ids:
            target = targets.get(group_id)
            if target is None:
                return TargetEvaluationResult(
                    False, "INVALID", "Incomplete joint target", group_ids=tuple(group_ids)
                )
            names.extend(str(name) for name in target.name)
            positions.extend(float(value) for value in target.position)
        return self.operator.evaluate_joint_target(
            JointTargetRequest(tuple(group_ids), JointState({"name": names, "position": positions}))
        )

    def evaluate_pose_target_set(
        self,
        pose_targets: Mapping[PlanningGroupID, Pose],
        auxiliary_group_ids: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
    ) -> TargetEvaluationResult:
        stamped = {
            group_id: PoseStamped(
                frame_id="world", position=pose.position, orientation=pose.orientation
            )
            for group_id, pose in pose_targets.items()
        }
        return self.operator.evaluate_pose_target(
            PoseTargetRequest(stamped, tuple(auxiliary_group_ids), _copy_joint_state(seed))
        )

    def cancel(self) -> bool:
        return self.operator.cancel()

    def clear_planned_path(self) -> bool:
        return self.operator.clear_plan()

    def plan_to_selected_joints(
        self, group_ids: Sequence[PlanningGroupID], targets: Mapping[PlanningGroupID, JointState]
    ) -> bool:
        names: list[str] = []
        positions: list[float] = []
        for group_id in group_ids:
            target = targets.get(group_id)
            if target is None:
                return False
            names.extend(str(name) for name in target.name)
            positions.extend(float(value) for value in target.position)
        plan = self.operator.plan_to_joints(
            JointTargetRequest(tuple(group_ids), JointState({"name": names, "position": positions}))
        )
        self.state.plan_state.plan = plan
        return plan is not None

    def preview_path(self) -> bool:
        plan = self.state.plan_state.plan
        return plan is not None and self.operator.preview(plan)

    def execute(self) -> bool:
        plan = self.state.plan_state.plan
        return plan is not None and self.operator.execute(plan)

    def refresh(self) -> None:
        if self._closed:
            return
        robots = self.list_robots()
        groups = self.list_planning_groups()
        self.state.backend_status = (
            BackendConnectionStatus.READY if robots else BackendConnectionStatus.WAITING_FOR_ROBOT
        )
        if not self.state.selected_group_ids and groups and not self._default_group_initialized:
            first = next((group for group in groups if group.has_pose_target), groups[0])
            self.state.selected_group_ids = (first.id,)
            self.state.selected_robot = str(first.robot_name)
            self.state.target_status = TargetStatus.EMPTY
            self._default_group_initialized = True
        initialized_groups = set(self.state.group_joint_targets)
        self._initialize_selected_group_targets()
        if set(self.state.group_joint_targets) != initialized_groups:
            self._build_joint_sliders()
        self._sync_group_selector(groups)
        self._refresh_selected_robot_state()
        self._ensure_scene_controls()
        self._sync_target_ghost_visibility()
        self._sync_robot_display_dropdown()
        self._sync_robot_display_warning()
        self._sync_preset_dropdown()
        self._update_status_text()
        self._update_control_state()

    def _build(self) -> None:
        gui = self.server.gui
        folder = gui.add_folder("Manipulation Panel", expand_by_default=True)
        self._handles["panel_folder"] = folder
        with folder:
            self._build_panel_controls(gui)

    def _build_panel_controls(self, gui: GuiApi) -> None:
        self._handles["status"] = gui.add_markdown("### Status\n**State:** Ready")
        self._build_scene_controls(gui)
        self._handles["planning_groups_heading"] = gui.add_markdown(
            "### Planning Groups\nActive planning groups for pose goals, planning, and joint edits."
        )
        self._sync_group_selector(self.list_planning_groups())
        self._handles["target_heading"] = gui.add_markdown("### Target")
        preset_dropdown = gui.add_dropdown(
            "Preset",
            options=["Select preset...", "Current"],
            initial_value="Select preset...",
        )
        preset_dropdown.on_update(lambda event: self._apply_preset(event.target.value))
        self._handles["preset"] = preset_dropdown
        self._handles["target_summary"] = gui.add_markdown("Feasibility: `unknown`")
        self._handles["actions_heading"] = gui.add_markdown("### Actions")
        plan_button = gui.add_button("Plan", disabled=True, color=PRIMARY_ACTION_COLOR)
        plan_button.on_click(lambda _: self._submit_plan())
        self._handles["plan"] = plan_button
        self._handles["plan_controls_heading"] = gui.add_markdown("**Plan controls**")
        preview_button = gui.add_button("Preview", disabled=True)
        preview_button.on_click(lambda _: self._submit_preview())
        self._handles["preview"] = preview_button
        execute_button = gui.add_button("Execute", disabled=True)
        execute_button.on_click(lambda _: self._submit_execute())
        self._handles["execute"] = execute_button
        cancel_button = gui.add_button("Cancel")
        cancel_button.on_click(lambda _: self._submit_cancel())
        self._handles["cancel"] = cancel_button
        clear_button = gui.add_button("Clear plan")
        clear_button.on_click(lambda _: self._submit_clear())
        self._handles["clear"] = clear_button
        joint_controls = gui.add_folder("Joint Control", expand_by_default=False)
        self._handles["joint_control_folder"] = joint_controls
        self._build_joint_sliders()

    def _sync_group_selector(self, groups: list[PlanningGroup]) -> None:
        """Render source-order group toggle buttons without a robot dropdown."""
        selected = set(self.state.selected_group_ids)
        seen: set[str] = set()
        for group in sorted(
            groups, key=lambda item: (not bool(item.has_pose_target), str(item.id))
        ):
            group_id = str(group.id)
            key = f"group:{group_id}"
            seen.add(key)
            label = group_display_name(group)
            handle = self._handles.get(key)
            color = ACTIVE_GROUP_COLOR if group_id in selected else INACTIVE_GROUP_COLOR
            if handle is None:
                handle = self.server.gui.add_button(
                    label,
                    color=color,
                    hint="Click to toggle this planning group in the target set.",
                )

                def on_click(_event: object, selected_group_id: str = group_id) -> None:
                    self._toggle_group_selected(selected_group_id)

                handle.on_click(on_click)
                self._handles[key] = handle
            else:
                self._set_optional_handle_attr(handle, "label", label)
                self._set_optional_handle_attr(handle, "color", color)
        for key in [key for key in self._handles if key.startswith("group:") and key not in seen]:
            handle = self._handles.pop(key)
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()

    def _toggle_group_selected(self, group_id: str) -> None:
        groups = {str(group.id): group for group in self.list_planning_groups()}
        if group_id not in groups:
            return
        current = list(self.state.selected_group_ids)
        if group_id in current:
            current.remove(group_id)
        else:
            current.append(group_id)
        self.state.selected_group_ids = tuple(current)
        self.state.advance_selection_epoch()
        self._clear_invalidated_preview()
        first = groups.get(current[0]) if current else None
        self.state.selected_robot = None if first is None else str(first.robot_name)
        self._prune_inactive_group_state()
        self._initialize_selected_group_targets()
        self._build_joint_sliders()
        self.refresh()

    def _build_scene_controls(self, gui: GuiApi) -> None:
        if self.scene is None:
            return
        if self.scene.has_reference_grid():
            handle = gui.add_checkbox("Scene grid", initial_value=True)
            self._handles["scene_grid"] = handle
            handle.on_update(lambda event: self._set_scene_grid_visible(event.target.value))

        display_folder = gui.add_folder("Robot display", expand_by_default=True)
        self._handles["robot_display_folder"] = display_folder
        with display_folder:
            try:
                display_handle = gui.add_dropdown(
                    "Robot display",
                    options=ROBOT_DISPLAY_LABELS,
                    initial_value=self._robot_display_label(),
                    hint="Choose which primary robot geometry to show.",
                )
            except TypeError:
                # Keep compatibility with small fake GUI implementations and
                # older Viser releases that predate dropdown hints.
                display_handle = gui.add_dropdown(
                    "Robot display",
                    options=ROBOT_DISPLAY_LABELS,
                    initial_value=self._robot_display_label(),
                )
            display_handle.on_update(lambda event: self._set_robot_display_mode(event.target.value))
            self._handles["robot_display"] = display_handle
            try:
                warning_handle = gui.add_markdown(ROBOT_DISPLAY_COLLISION_WARNING, visible=False)
            except TypeError:
                # Keep compatibility with lightweight GUI fakes and older Viser
                # releases that do not accept the visible keyword.
                warning_handle = gui.add_markdown(ROBOT_DISPLAY_COLLISION_WARNING)
                self._set_optional_handle_attr(warning_handle, "visible", False)
            self._handles["robot_display_warning"] = warning_handle
            self._sync_robot_display_warning()

    def _robot_display_label(self) -> str:
        scene = self.scene
        if scene is None:
            return ROBOT_DISPLAY_LABELS[0]
        mode = scene.robot_display_mode.value
        if mode not in ROBOT_DISPLAY_MODES:
            return ROBOT_DISPLAY_LABELS[0]
        return ROBOT_DISPLAY_MODES[mode].value.title()

    def _set_robot_display_mode(self, label: str) -> None:
        if self._closed or self.scene is None:
            return
        scene = self.scene
        mode = str(label).lower()
        if mode not in ROBOT_DISPLAY_MODES:
            return
        scene.robot_display_mode = ROBOT_DISPLAY_MODES[mode]
        self._sync_robot_display_dropdown()
        self._sync_robot_display_warning()

    def _sync_robot_display_dropdown(self) -> None:
        handle = self._handles.get("robot_display")
        scene = self.scene
        if handle is None or self._closed or scene is None:
            return
        self._set_optional_handle_attr(handle, "value", self._robot_display_label())

    def _sync_robot_display_warning(self) -> None:
        handle = self._handles.get("robot_display_warning")
        scene = self.scene
        if handle is None or self._closed or scene is None:
            return
        mode = scene.robot_display_mode
        has_collision = scene.collision_geometry_available
        visible = mode in {RobotDisplayMode.COLLISION, RobotDisplayMode.BOTH} and not has_collision
        self._set_optional_handle_attr(handle, "visible", visible)

    def _set_scene_grid_visible(self, visible: bool) -> None:
        if self._closed:
            return
        if self.scene is None:
            return
        self.scene.set_reference_grid_visible(bool(visible))

    def _refresh_selected_robot_state(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            self.state.current_joints = None
            self.state.manipulation_state = self.get_module_state()
            return
        current = self.get_current_joint_state(robot_name)
        self.state.current_joints = list(current.position) if current is not None else None
        self.state.manipulation_state = self.get_module_state()
        adapter_error = self.get_error()
        if adapter_error:
            self.state.error = adapter_error

    def _ensure_scene_controls(self) -> None:
        if self.scene is None:
            return
        groups = self._groups_by_id()
        pose_group_ids = tuple(
            group_id
            for group_id in self.state.selected_group_ids
            if (group := groups.get(group_id)) is not None and group.has_pose_target
        )
        for key in [key for key in self._handles if key.startswith("ee_control:")]:
            if key.removeprefix("ee_control:") not in pose_group_ids:
                self.scene.remove_target_controls(key.removeprefix("ee_control:"))
                self._handles.pop(key, None)
        for group_id in pose_group_ids:
            group = groups[group_id]

            def on_transform_update(
                target: TransformControlsHandle,
                selected_group_id: PlanningGroupID = group_id,
            ) -> None:
                self._on_transform_update(selected_group_id, target)

            control = self.scene.ensure_target_controls(
                str(group_id),
                on_transform_update,
            )
            if control is not None:
                self._handles[f"ee_control:{group_id}"] = control
            pose = self.state.pose_targets.get(group_id)
            if pose is not None:
                self._suppress_target_callbacks = True
                try:
                    self.scene.set_target_pose(str(group_id), pose)
                finally:
                    self._suppress_target_callbacks = False

    def _build_joint_sliders(self) -> None:
        gui = self.server.gui
        self._clear_joint_sliders()
        if not self.state.selected_group_ids:
            return
        joint_folder = self._handles.get("joint_control_folder")
        if joint_folder is not None:
            folder = cast("GuiFolderHandle", joint_folder)
            with folder:
                self._build_joint_slider_handles(gui)
            return
        self._build_joint_slider_handles(gui)

    def _build_joint_slider_handles(self, gui: GuiApi) -> None:
        for group_id in self.state.selected_group_ids:
            group = self._groups_by_id().get(group_id)
            if group is None:
                continue
            config = self.get_robot_config(group.robot_name)
            target = self.state.group_joint_targets.get(group_id)
            if config is None or target is None:
                continue
            config_indexes = {str(name): index for index, name in enumerate(config.joint_names)}
            for _global_name, local_name, value in zip(
                group.joint_names, group.local_joint_names, target.position, strict=True
            ):
                index = config_indexes.get(str(local_name))
                lower, upper = DEFAULT_JOINT_LIMITS
                if index is not None and config.joint_limits_lower is not None:
                    lower = config.joint_limits_lower[index]
                if index is not None and config.joint_limits_upper is not None:
                    upper = config.joint_limits_upper[index]
                key = (group_id, str(local_name))
                handle = gui.add_slider(
                    f"{group_id}/{local_name}",
                    min=float(lower),
                    max=float(upper),
                    step=0.001,
                    initial_value=float(value),
                )

                def on_slider_update(
                    _event: object,
                    selected_group_id: PlanningGroupID = group_id,
                    name: str = str(local_name),
                ) -> None:
                    self._on_joint_slider_update(selected_group_id, name)

                handle.on_update(on_slider_update)
                self._joint_sliders[key] = handle

    def _clear_joint_sliders(self) -> None:
        for handle in self._joint_sliders.values():
            try:
                handle.remove()
            except AttributeError:
                pass
        self._joint_sliders.clear()

    def _groups_by_id(self) -> dict[PlanningGroupID, PlanningGroup]:
        return {group.id: group for group in self.list_planning_groups()}

    def _selected_robot_names(self) -> tuple[str, ...]:
        groups = self._groups_by_id()
        return tuple(
            dict.fromkeys(
                str(groups[group_id].robot_name)
                for group_id in self.state.selected_group_ids
                if group_id in groups
            )
        )

    def _stale_robot_names(self, group_ids: tuple[PlanningGroupID, ...]) -> tuple[str, ...]:
        """Return every affected robot whose monitored joint state is stale."""
        groups = self._groups_by_id()
        robot_names = tuple(
            dict.fromkeys(
                str(groups[group_id].robot_name) for group_id in group_ids if group_id in groups
            )
        )
        return tuple(name for name in robot_names if self.is_state_stale(name))

    def _state_values_by_local_name(self, state: JointState | None) -> dict[str, float]:
        if state is None or len(state.name) != len(state.position):
            return {}
        return {
            str(name): float(value) for name, value in zip(state.name, state.position, strict=True)
        }

    def _local_values_for_robot(
        self, robot_name: str, state: JointState | None
    ) -> dict[str, float]:
        config = self.get_robot_config(robot_name)
        if config is None or state is None or len(state.name) != len(state.position):
            return {}
        raw = self._state_values_by_local_name(state)
        values: dict[str, float] = {}
        for local_name in config.joint_names:
            global_name = f"{robot_name}/{local_name}"
            if local_name in raw:
                values[local_name] = raw[local_name]
            elif global_name in raw:
                values[local_name] = raw[global_name]
        return values

    def _initialize_selected_group_targets(self) -> None:
        for group_id in self.state.selected_group_ids:
            if group_id in self.state.group_joint_targets:
                continue
            group = self._groups_by_id().get(group_id)
            if group is None:
                continue
            if self.is_state_stale(group.robot_name):
                continue
            values = self._local_values_for_robot(
                str(group.robot_name), self.get_current_joint_state(group.robot_name)
            )
            if any(str(name) not in values for name in group.local_joint_names):
                continue
            self.state.group_joint_targets[group_id] = JointState(
                {
                    "name": list(group.joint_names),
                    "position": [float(values[str(name)]) for name in group.local_joint_names],
                }
            )
            if group.has_pose_target and group_id not in self.state.pose_targets:
                pose = self.get_group_ee_pose(group_id)
                if pose is not None:
                    self.state.pose_targets[group_id] = pose
                    self.state.group_poses[group_id] = pose
                    if self.state.cartesian_target is None:
                        self.state.cartesian_target = pose
        self._refresh_target_joints_from_groups()

    def _prune_inactive_group_state(self) -> None:
        selected = set(self.state.selected_group_ids)
        for values in (
            self.state.pose_targets,
            self.state.group_joint_targets,
            self.state.group_poses,
        ):
            for group_id in tuple(values):
                if group_id not in selected:
                    values.pop(group_id)
        self._refresh_target_joints_from_groups()

    def _refresh_target_joints_from_groups(self) -> None:
        names: list[str] = []
        positions: list[float] = []
        for group_id in self.state.selected_group_ids:
            target = self.state.group_joint_targets.get(group_id)
            if target is not None:
                names.extend(str(name) for name in target.name)
                positions.extend(float(value) for value in target.position)
        self.state.target_joints = (
            JointState({"name": names, "position": positions}) if names else None
        )

    def _active_pose_targets(self) -> dict[PlanningGroupID, Pose]:
        return {
            group_id: self.state.pose_targets[group_id]
            for group_id in self.state.selected_group_ids
            if group_id in self.state.pose_targets
        }

    def _preset_values_by_local_name(self, preset: str, robot_name: str) -> dict[str, float]:
        if preset == "Current":
            state = self.get_current_joint_state(robot_name)
        elif preset == "Init":
            state = self.get_init_joints(robot_name)
        else:
            config = self.get_robot_config(robot_name)
            if config is None:
                return {}
            return {
                str(name): float(value)
                for name, value in zip(config.joint_names, config.home_joints or [], strict=False)
            }
        return self._local_values_for_robot(robot_name, state)

    def _remove_panel_handles(self) -> None:
        for key, handle in list(self._handles.items()):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
            self._handles.pop(key, None)

    def _sync_preset_dropdown(self) -> None:
        handle = self._handles.get("preset")
        if handle is None or not self.state.selected_group_ids:
            return
        options = ["Select preset..."]
        selected_robots = self._selected_robot_names()
        if any(self.get_init_joints(robot_name) is not None for robot_name in selected_robots):
            options.append("Init")
        options.append("Current")
        if any(
            (config := self.get_robot_config(robot_name)) is not None
            and config.home_joints is not None
            for robot_name in selected_robots
        ):
            options.append("Home")
        for attr in ("options", "values"):
            if hasattr(handle, attr):
                try:
                    self._set_optional_handle_attr(handle, attr, options)
                except Exception:
                    logger.warning("Could not set preset dropdown %s", attr, exc_info=True)

    def _apply_preset(self, preset: str) -> None:
        if self._closed:
            return
        if preset not in {"Current", "Init", "Home"}:
            return
        targets: dict[PlanningGroupID, JointState] = {}
        slider_values: list[tuple[PlanningGroupID, tuple[str, ...], list[float]]] = []
        for group_id in self.state.selected_group_ids:
            group = self._groups_by_id().get(group_id)
            if group is None:
                self._set_recoverable_error(f"Unknown planning group: {group_id}")
                return
            if preset == "Current" and self.is_state_stale(group.robot_name):
                self._set_recoverable_error(
                    f"Cannot apply Current preset without fresh telemetry for: {group.robot_name}"
                )
                return
            values = self._preset_values_by_local_name(preset, str(group.robot_name))
            missing = [str(name) for name in group.local_joint_names if str(name) not in values]
            if missing:
                self._set_recoverable_error(
                    f"Cannot apply {preset} preset: missing joints for {group_id}: {', '.join(missing)}"
                )
                return
            positions = [float(values[str(name)]) for name in group.local_joint_names]
            targets[group_id] = JointState({"name": list(group.joint_names), "position": positions})
            slider_values.append((group_id, group.local_joint_names, positions))
        self.state.group_joint_targets.update(targets)
        if any(
            (group_id, str(local_name)) not in self._joint_sliders
            for group_id, local_names, _positions in slider_values
            for local_name in local_names
        ):
            self._build_joint_sliders()
        for group_id, local_names, positions in slider_values:
            self._set_group_slider_values(group_id, local_names, positions)
        self._refresh_target_joints_from_groups()
        self._submit_joint_target_evaluation()
        self.refresh()

    def _set_group_slider_values(
        self, group_id: PlanningGroupID, local_names: tuple[str, ...], values: list[float]
    ) -> None:
        self._suppress_target_callbacks = True
        try:
            for local_name, value in zip(local_names, values, strict=True):
                handle = self._joint_sliders.get((group_id, str(local_name)))
                if handle is not None:
                    handle.value = float(value)
        finally:
            self._suppress_target_callbacks = False

    def _target_set_from_sliders(self) -> dict[PlanningGroupID, JointState] | None:
        targets: dict[PlanningGroupID, JointState] = {}
        for group_id in self.state.selected_group_ids:
            group = self._groups_by_id().get(group_id)
            if group is None:
                self._set_error(f"Unknown planning group: {group_id}")
                return None
            positions: list[float] = []
            for local_name in group.local_joint_names:
                handle = self._joint_sliders.get((group_id, str(local_name)))
                if handle is None:
                    self._set_error(f"Missing target slider for {group_id}/{local_name}")
                    return None
                positions.append(float(handle.value))
            targets[group_id] = JointState({"name": list(group.joint_names), "position": positions})
        return targets

    def _on_joint_slider_update(self, _group_id: PlanningGroupID, _local_name: str) -> None:
        if self._closed:
            return
        if self._suppress_target_callbacks:
            return
        self._submit_joint_target_evaluation()

    def _on_transform_update(
        self, group_id: PlanningGroupID, target: TransformControlsHandle
    ) -> None:
        if self._closed:
            return
        if self._suppress_target_callbacks or group_id not in self.state.selected_group_ids:
            return
        pose = self._pose_from_transform_target(target)
        if pose is None:
            return
        self.state.cartesian_target = pose
        self.state.pose_targets[group_id] = pose
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="cartesian",
                selection_epoch=self.state.selection_epoch,
                group_ids=self.state.selected_group_ids,
                auxiliary_group_ids=tuple(
                    selected_group_id
                    for selected_group_id in self.state.selected_group_ids
                    if selected_group_id not in self._active_pose_targets()
                ),
                joints=(
                    None
                    if self.state.target_joints is None
                    else JointState(self.state.target_joints)
                ),
                pose_targets=dict(self._active_pose_targets()),
            )
        )
        self.refresh()

    def _submit_joint_target_evaluation(self) -> None:
        targets = self._target_set_from_sliders()
        if targets is None:
            return
        self.state.group_joint_targets = targets
        self._refresh_target_joints_from_groups()
        self._move_joint_target_visuals(targets)
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="joints",
                selection_epoch=self.state.selection_epoch,
                group_ids=self.state.selected_group_ids,
                joint_targets=dict(targets),
            )
        )
        self.refresh()

    def _move_joint_target_visuals(self, targets: Mapping[PlanningGroupID, JointState]) -> None:
        """Optimistically move target visuals before collision/feasibility returns."""
        if self.scene is None:
            return
        for robot_name, state in self._target_ghost_states(targets).items():
            config = self.get_robot_config(robot_name)
            robot_id = self.robot_id_for_name(robot_name)
            if config is not None and robot_id is not None:
                self.scene.set_target_joints(str(robot_id), config.joint_names, state.position)

    def _target_ghost_states(
        self, targets: Mapping[PlanningGroupID, JointState]
    ) -> dict[str, JointState]:
        groups = self._groups_by_id()
        merged: dict[str, dict[str, float]] = {}
        configs: dict[str, tuple[str, ...]] = {}
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            target = targets.get(group_id)
            if group is None or target is None:
                continue
            robot_name = str(group.robot_name)
            config = self.get_robot_config(robot_name)
            current = self.get_current_joint_state(robot_name)
            if config is None or current is None:
                continue
            values = self._local_values_for_robot(robot_name, current)
            target_raw = self._state_values_by_local_name(target)
            for local_name, global_name in zip(
                group.local_joint_names, group.joint_names, strict=True
            ):
                if str(global_name) in target_raw:
                    values[str(local_name)] = target_raw[str(global_name)]
                elif str(local_name) in target_raw:
                    values[str(local_name)] = target_raw[str(local_name)]
            if all(name in values for name in config.joint_names):
                merged[robot_name] = values
                configs[robot_name] = tuple(config.joint_names)
        return {
            robot_name: JointState(
                {"name": list(joint_names), "position": [values[name] for name in joint_names]}
            )
            for robot_name, values in merged.items()
            for joint_names in (configs[robot_name],)
        }

    def _sync_target_ghost_visibility(self) -> None:
        if self.scene is None:
            return
        active_robot_ids = {
            str(robot_id)
            for group_id in self.state.selected_group_ids
            if (group := self._groups_by_id().get(group_id)) is not None
            and group.has_pose_target
            and (robot_id := self.robot_id_for_name(group.robot_name)) is not None
        }
        for _robot_name, robot_id, _config in self.robot_items():
            self.scene.set_target_active(str(robot_id), str(robot_id) in active_robot_ids)

    def _handle_target_evaluation_request(
        self, request: TargetEvaluationRequest
    ) -> TargetEvaluationResult:
        if request.source == "cartesian":
            if not request.pose_targets:
                return TargetEvaluationResult(False, "INVALID", "No pose target")
            return self.evaluate_pose_target_set(
                request.pose_targets, request.auxiliary_group_ids, request.joints
            )
        if not request.joint_targets:
            return TargetEvaluationResult(False, "INVALID", "No joint target")
        return self.evaluate_joint_target_set(request.group_ids, request.joint_targets)

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: TargetEvaluationResult
    ) -> None:
        if self._closed:
            return
        if (
            request.sequence_id != self.state.latest_sequence_id
            or request.selection_epoch != self.state.selection_epoch
            or request.group_ids != self.state.selected_group_ids
        ):
            return
        collision_free = result.collision_free
        success = result.success
        self.state.feasibility.status = self._feasibility_status(result, success, collision_free)
        self.state.feasibility.message = result.message
        self.state.target_status = (
            TargetStatus.FEASIBLE if success and collision_free else TargetStatus.INFEASIBLE
        )
        self.state.error = "" if success and collision_free else self.state.feasibility.message
        if result.target_joints is not None:
            self.state.target_joints = JointState(result.target_joints)
            self._split_target_joints_by_group(result.target_joints)
        self.state.group_poses = {
            str(group_id): pose
            for group_id, pose in result.group_poses.items()
            if isinstance(pose, Pose)
        }
        if request.source == "joints":
            self._sync_pose_targets_from_group_poses()
        else:
            self._sync_controls_from_targets()
        self._update_target_visual_state()
        self.refresh()

    def _sync_controls_from_targets(self) -> None:
        for group_id, target in self.state.group_joint_targets.items():
            group = self._groups_by_id().get(group_id)
            if group is not None:
                self._set_group_slider_values(
                    group_id, group.local_joint_names, list(target.position)
                )
        self._move_joint_target_visuals(self.state.group_joint_targets)

    def _split_target_joints_by_group(self, target_joints: JointState) -> None:
        if len(target_joints.name) != len(target_joints.position):
            return
        positions = {
            str(name): float(value)
            for name, value in zip(target_joints.name, target_joints.position, strict=True)
        }
        for group_id in self.state.selected_group_ids:
            group = self._groups_by_id().get(group_id)
            if group is None or any(str(name) not in positions for name in group.joint_names):
                continue
            self.state.group_joint_targets[group_id] = JointState(
                {
                    "name": list(group.joint_names),
                    "position": [positions[str(name)] for name in group.joint_names],
                }
            )

    def _sync_pose_targets_from_group_poses(self) -> None:
        groups = self._groups_by_id()
        active_group_ids: list[PlanningGroupID] = []
        for group_id, pose in self.state.group_poses.items():
            group = groups.get(group_id)
            if group is None or not group.has_pose_target:
                continue
            if group_id not in self.state.selected_group_ids:
                continue
            self.state.pose_targets[group_id] = pose
            active_group_ids.append(group_id)
        if self.scene is None:
            return
        self._suppress_target_callbacks = True
        try:
            for group_id in active_group_ids:
                self.scene.set_target_pose(str(group_id), self.state.pose_targets[group_id])
        finally:
            self._suppress_target_callbacks = False

    def _update_status_text(self) -> None:
        current = self.state.current_joints
        status_label = self.state.error or self.state.module_state
        status = [
            "### Status",
            f"**State:** {status_label}",
            f"Target: `{self.state.target_status.value}` · Plan: `{self.state.plan_state.status.value}`",
        ]
        stale_robots = self._stale_robot_names(self.state.selected_group_ids)
        if self.state.selected_group_ids:
            stale_detail = "False" if not stale_robots else f"True ({', '.join(stale_robots)})"
            status.append(f"State stale: `{stale_detail}`")
        if current is not None:
            status.append(f"Current joints: `{[round(v, 3) for v in current]}`")
        if self.state.last_result:
            status.append(f"Last result: `{self.state.last_result}`")
        self._set_handle_value("status", "\n\n".join(status))
        self._set_handle_value(
            "target_summary",
            f"Feasibility: `{self.state.feasibility.status.value}`",
        )

    def _update_control_state(self) -> None:
        self._set_disabled("plan", not self.state.can_plan())
        self._set_disabled("preview", not self.state.can_preview())
        self._set_disabled(
            "execute",
            not self._can_execute(),
        )
        can_cancel = self.state.can_cancel()
        self._set_disabled("cancel", not can_cancel)
        self._set_visible("cancel", can_cancel)
        self._update_target_visual_state()

    def _update_target_visual_state(self) -> None:
        if self.scene is None:
            return
        feasible = self.state.feasibility.status == FeasibilityStatus.FEASIBLE
        groups = self._groups_by_id()
        selected_groups = tuple(
            (group_id, groups[group_id])
            for group_id in self.state.selected_group_ids
            if group_id in groups
        )
        for group_id, group in selected_groups:
            if group.has_pose_target:
                self.scene.set_target_control_visual_state(str(group_id), feasible)
        robot_ids = tuple(
            dict.fromkeys(
                str(robot_id)
                for _group_id, group in selected_groups
                if (robot_id := self.robot_id_for_name(str(group.robot_name))) is not None
            )
        )
        for robot_id in robot_ids:
            self.scene.set_target_robot_visual_state(robot_id, feasible)

    def _can_execute(self) -> bool:
        return self.state.can_execute()

    def _submit_plan(self) -> None:
        if self._closed:
            return
        if not self.state.can_plan():
            self._set_recoverable_error(
                "Cannot plan until target is feasible and manipulation is idle"
            )
            return
        group_ids = self.state.selected_group_ids
        selection_epoch = self.state.selection_epoch
        target_sequence_id = self.state.latest_sequence_id
        targets = self._target_set_from_sliders()
        if targets is None:
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                return
            self.state.action_status = ActionStatus.RUNNING
            self.state.plan_state.status = PlanStatus.PLANNING
            stale_robots = self._stale_robot_names(group_ids)
            if stale_robots:
                if not self._operation_is_current(
                    operation_id, selection_epoch, target_sequence_id
                ):
                    self._finish_operation(
                        "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                    )
                    return
                self.state.plan_state.status = PlanStatus.STALE
                self.state.error = "Cannot plan without fresh telemetry for: " + ", ".join(
                    stale_robots
                )
                self._finish_operation(
                    "plan=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            ok = self.plan_to_selected_joints(group_ids, targets)
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            if ok:
                self.state.plan_state.status = PlanStatus.FRESH
                self.state.plan_state.group_ids = group_ids
                self.state.plan_state.target_sequence_id = target_sequence_id
            else:
                self.state.plan_state.status = PlanStatus.FAILED
                self.state.plan_state.plan = None
            self._finish_operation(
                f"plan_to_joints={ok}",
                operation_id=operation_id,
                selection_epoch=selection_epoch,
            )

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_preview(self) -> None:
        if self._closed:
            return
        if not self.state.can_preview():
            self._set_recoverable_error("No fresh plan to preview")
            return
        selection_epoch = self.state.selection_epoch
        operation_id = self._next_operation_id()
        self.state.action_status = ActionStatus.PREVIEWING
        self.refresh()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch):
                return
            ok = self.preview_path()
            self._finish_operation(
                f"preview={ok}", operation_id=operation_id, selection_epoch=selection_epoch
            )

        self._operation_worker.submit(
            operation,
            timeout_seconds=self.config.preview_request_timeout,
            on_error=lambda message: self._set_operation_error(message, operation_id),
        )

    def _clear_invalidated_preview(self) -> None:
        if self.state.action_status == ActionStatus.PREVIEWING:
            self._operation_sequence_id += 1
            self.state.action_status = ActionStatus.IDLE
            self.state.last_result = "preview=False"

    def _submit_execute(self) -> None:
        if self._closed:
            return
        if not self._can_execute():
            self._set_recoverable_error("Cannot execute: require feasible fresh plan")
            return
        selection_epoch = self.state.selection_epoch
        group_ids = self.state.selected_group_ids
        target_sequence_id = self.state.latest_sequence_id
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                return
            if (
                self.state.plan_state.group_ids != group_ids
                or self.state.plan_state.target_sequence_id != target_sequence_id
            ):
                self.state.plan_state.status = PlanStatus.STALE
                self._finish_operation(
                    "execute=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "execute=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            self.state.action_status = ActionStatus.EXECUTING
            self.state.plan_state.status = PlanStatus.EXECUTING
            ok = self.execute()
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "execute=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            if not ok:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(
                f"execute={ok}", operation_id=operation_id, selection_epoch=selection_epoch
            )

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_cancel(self) -> None:
        if self._closed:
            return
        cancelled_action = self.state.action_status
        operation_id = self._next_operation_id()
        if not self._operation_is_current(operation_id):
            return
        self.state.action_status = ActionStatus.CANCELLING
        self._mark_cancelled_plan_state(cancelled_action)
        self._restart_operation_worker()
        try:
            ok = self.cancel()
        except Exception as e:
            self._set_operation_error(str(e), operation_id)
            return
        self._finish_operation(f"cancel={ok}", operation_id=operation_id)

    def _mark_cancelled_plan_state(self, cancelled_action: ActionStatus) -> None:
        if self.state.plan_state.status == PlanStatus.PLANNING:
            self.state.plan_state.status = PlanStatus.FAILED
        elif (
            cancelled_action == ActionStatus.EXECUTING
            or self.state.plan_state.status == PlanStatus.EXECUTING
        ):
            self.state.plan_state.status = PlanStatus.STALE

    def _restart_operation_worker(self) -> None:
        self._operation_worker.stop(timeout=0.0)
        self._operation_worker = OperationWorker(self._set_error)
        self._operation_worker.start()

    def _submit_clear(self) -> None:
        if self._closed:
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.CLEARING_PLAN
            ok = self.clear_planned_path()
            if not self._operation_is_current(operation_id):
                return
            self.state.plan_state = PanelPlanState()
            self._finish_operation(f"clear={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _next_operation_id(self) -> int:
        self._operation_sequence_id += 1
        return self._operation_sequence_id

    def _operation_is_current(
        self,
        operation_id: int,
        selection_epoch: int | None = None,
        target_sequence_id: int | None = None,
    ) -> bool:
        return (
            not self._closed
            and operation_id == self._operation_sequence_id
            and (selection_epoch is None or selection_epoch == self.state.selection_epoch)
            and (target_sequence_id is None or target_sequence_id == self.state.latest_sequence_id)
        )

    def _finish_operation(
        self,
        result: str,
        *,
        clear_error: bool = True,
        operation_id: int | None = None,
        selection_epoch: int | None = None,
    ) -> None:
        if self._closed or (
            operation_id is not None
            and not self._operation_is_current(operation_id, selection_epoch)
        ):
            return
        self.state.action_status = ActionStatus.IDLE
        if clear_error:
            self.state.error = ""
        self.state.last_result = result
        self.refresh()

    def _set_operation_error(self, message: str, operation_id: int) -> None:
        if self._operation_is_current(operation_id):
            self._operation_sequence_id += 1
            self._set_error(message)

    def _set_recoverable_error(self, message: str) -> None:
        if self._closed:
            return
        self.state.error = message
        self.refresh()

    def _set_error(self, message: str) -> None:
        if self._closed:
            return
        self.state.action_status = ActionStatus.FAILED
        self.state.error = message
        self.refresh()

    def _set_handle_value(self, key: str, value: str) -> None:
        handle = self._handles.get(key)
        if isinstance(handle, GuiMarkdownHandle):
            self._set_optional_handle_attr(handle, "value", value)

    def _set_disabled(self, key: str, disabled: bool) -> None:
        handle = self._handles.get(key)
        if isinstance(handle, GuiButtonHandle):
            self._set_optional_handle_attr(handle, "disabled", disabled)

    def _set_visible(self, key: str, visible: bool) -> None:
        handle = self._handles.get(key)
        if handle is not None:
            self._set_optional_handle_attr(handle, "visible", visible)

    @staticmethod
    def _set_optional_handle_attr(handle: object, attr: str, value: object) -> None:
        setattr(handle, attr, value)

    def _pose_from_transform_target(self, target: TransformControlsHandle) -> Pose | None:
        px, py, pz = (float(value) for value in target.position)
        qw, qx, qy, qz = (float(value) for value in target.wxyz)
        return Pose({"position": [px, py, pz], "orientation": [qx, qy, qz, qw]})

    def _feasibility_status(
        self, result: TargetEvaluationResult, success: bool, collision_free: bool
    ) -> FeasibilityStatus:
        status = result.status.upper()
        if success and collision_free:
            return FeasibilityStatus.FEASIBLE
        if status in {"COLLISION", "COLLISION_AT_START", "COLLISION_AT_GOAL"}:
            return FeasibilityStatus.COLLISION
        if status in {"NO_SOLUTION", "SINGULARITY", "JOINT_LIMITS", "TIMEOUT"}:
            return FeasibilityStatus.IK_FAILED
        return FeasibilityStatus.INVALID
