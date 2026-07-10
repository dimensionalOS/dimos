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

from collections.abc import Mapping
from typing import TYPE_CHECKING, TypeAlias, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.visualization.types import TargetEvaluation, TargetSetEvaluation
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.panel_backend import GroupPanelBackend
from dimos.manipulation.visualization.viser.runtime import VISER_INSTALL_HINT
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
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor

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


class ViserPanelGui:
    """Optional operator panel with parity for the original cc/viser-vis panel."""

    def __init__(
        self,
        server: ViserServer,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule,
        config: ViserVisualizationConfig,
        scene: ViserManipulationScene | None = None,
    ) -> None:
        self.server = server
        self.adapter = GroupPanelBackend(world_monitor, manipulation_module)
        self._world_monitor = world_monitor
        self._manipulation_module = manipulation_module
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

    def refresh(self) -> None:
        if self._closed:
            return
        robots = self.adapter.list_robots()
        groups = self.adapter.list_planning_groups()
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
            "### Planning Groups\nActive MoveIt group for pose goal, planning, and joint edits."
        )
        self._sync_group_selector(self.adapter.list_planning_groups())
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
            label = (
                str(group.robot_name)
                if str(group.group_name) == "manipulator"
                else f"{group.robot_name} {group.group_name}"
            )
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
        groups = {str(group.id): group for group in self.adapter.list_planning_groups()}
        if group_id not in groups:
            return
        current = list(self.state.selected_group_ids)
        if group_id in current:
            current.remove(group_id)
        else:
            current.append(group_id)
        self.state.selected_group_ids = tuple(current)
        self.state.advance_selection_epoch()
        if self.scene is not None:
            self.scene.cancel_preview_animation()
        first = groups.get(current[0]) if current else None
        self.state.selected_robot = None if first is None else str(first.robot_name)
        self._prune_inactive_group_state()
        self._initialize_selected_group_targets()
        self._build_joint_sliders()
        self.refresh()

    def _build_scene_controls(self, gui: GuiApi) -> None:
        if self.scene is None:
            return
        if not self.scene.has_reference_grid():
            return
        handle = gui.add_checkbox("Scene grid", initial_value=True)
        self._handles["scene_grid"] = handle
        handle.on_update(lambda event: self._set_scene_grid_visible(event.target.value))

    def _set_scene_grid_visible(self, visible: bool) -> None:
        if self._closed:
            return
        if self.scene is None:
            return
        self.scene.set_reference_grid_visible(bool(visible))

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
            config = self.adapter.get_robot_config(group.robot_name)
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
        return {group.id: group for group in self.adapter.list_planning_groups()}

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
        return tuple(name for name in robot_names if self.adapter.is_state_stale(name))

    def _state_values_by_local_name(self, state: JointState | None) -> dict[str, float]:
        if state is None or len(state.name) != len(state.position):
            return {}
        return {
            str(name).rsplit("/", 1)[-1]: float(value)
            for name, value in zip(state.name, state.position, strict=True)
        }

    def _initialize_selected_group_targets(self) -> None:
        for group_id in self.state.selected_group_ids:
            if group_id in self.state.group_joint_targets:
                continue
            group = self._groups_by_id().get(group_id)
            if group is None:
                continue
            if self.adapter.is_state_stale(group.robot_name):
                continue
            values = self._state_values_by_local_name(
                self.adapter.get_current_joint_state(group.robot_name)
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
                try:
                    pose = self._world_monitor.get_group_ee_pose(group.id)
                except ValueError:
                    pose = None
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
            self.state.group_diagnostics,
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
            state = self.adapter.get_current_joint_state(robot_name)
        elif preset == "Init":
            state = self.adapter.get_init_joints(robot_name)
        else:
            config = self.adapter.get_robot_config(robot_name)
            if config is None:
                return {}
            return {
                str(name): float(value)
                for name, value in zip(config.joint_names, config.home_joints or [], strict=False)
            }
        return self._state_values_by_local_name(state)

    def _remove_panel_handles(self) -> None:
        for key, handle in list(self._handles.items()):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
            self._handles.pop(key, None)

    def _select_robot(self, robot_name: str) -> None:
        if self._closed:
            return
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
                    self._set_optional_handle_attr(handle, attr, options)
                except Exception:
                    logger.warning("Could not set robot dropdown %s", attr, exc_info=True)
        if hasattr(handle, "value") and self.state.selected_robot in robots:
            try:
                self._set_optional_handle_attr(handle, "value", self.state.selected_robot)
            except Exception:
                logger.warning("Could not set robot dropdown value", exc_info=True)

    def _sync_preset_dropdown(self) -> None:
        handle = self._handles.get("preset")
        if handle is None or not self.state.selected_group_ids:
            return
        options = ["Select preset..."]
        selected_robots = self._selected_robot_names()
        if any(
            self.adapter.get_init_joints(robot_name) is not None for robot_name in selected_robots
        ):
            options.append("Init")
        options.append("Current")
        if any(
            (config := self.adapter.get_robot_config(robot_name)) is not None
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
            if preset == "Current" and self.adapter.is_state_stale(group.robot_name):
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
        complete = self.adapter.complete_states_for_targets(self.state.selected_group_ids, targets)
        if complete is None:
            return
        for robot_name, state in complete.items():
            config = self.adapter.get_robot_config(robot_name)
            robot_id = self.adapter.robot_id_for_name(robot_name)
            if config is not None and robot_id is not None:
                self.scene.set_target_joints(str(robot_id), config.joint_names, state.position)

    def _sync_target_ghost_visibility(self) -> None:
        if self.scene is None:
            return
        active_robot_ids = {
            str(robot_id)
            for group_id in self.state.selected_group_ids
            if (group := self._groups_by_id().get(group_id)) is not None
            and group.has_pose_target
            and (robot_id := self.adapter.robot_id_for_name(group.robot_name)) is not None
        }
        for _robot_name, robot_id, _config in self.adapter.robot_items():
            self.scene.set_target_active(str(robot_id), str(robot_id) in active_robot_ids)

    def _handle_target_evaluation_request(
        self, request: TargetEvaluationRequest
    ) -> TargetEvaluation | TargetSetEvaluation:
        if request.source == "cartesian":
            if not request.pose_targets:
                return {"success": False, "status": "INVALID", "message": "No pose target"}
            return self.adapter.evaluate_pose_target_set(
                request.pose_targets, request.auxiliary_group_ids, request.joints
            )
        if not request.joint_targets:
            return {"success": False, "status": "INVALID", "message": "No joint target"}
        return self.adapter.evaluate_joint_target_set(request.group_ids, request.joint_targets)

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: TargetEvaluation | TargetSetEvaluation
    ) -> None:
        if self._closed:
            return
        if (
            request.sequence_id != self.state.latest_sequence_id
            or request.selection_epoch != self.state.selection_epoch
            or request.group_ids != self.state.selected_group_ids
        ):
            return
        collision_free = bool(result.get("collision_free", False))
        success = bool(result.get("success", False))
        self.state.feasibility.status = self._feasibility_status(result, success, collision_free)
        self.state.feasibility.message = str(result.get("message", ""))
        self.state.target_status = (
            TargetStatus.FEASIBLE if success and collision_free else TargetStatus.INFEASIBLE
        )
        self.state.error = "" if success and collision_free else self.state.feasibility.message
        target_joints = result.get("target_joints") or result.get("joint_state")
        if isinstance(target_joints, JointState):
            self.state.target_joints = JointState(target_joints)
            self._split_target_joints_by_group(target_joints)
            if success and collision_free:
                self.state.last_valid_target_joints = JointState(target_joints)
        diagnostics = result.get("group_diagnostics", {})
        if isinstance(diagnostics, dict):
            self.state.group_diagnostics = {
                str(group_id): str(message) for group_id, message in diagnostics.items()
            }
        poses = result.get("group_poses", {})
        if isinstance(poses, dict):
            self.state.group_poses = {
                str(group_id): pose for group_id, pose in poses.items() if isinstance(pose, Pose)
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
                if (robot_id := self.adapter.robot_id_for_name(str(group.robot_name))) is not None
            )
        )
        for robot_id in robot_ids:
            self.scene.set_target_robot_visual_state(robot_id, feasible)

    def _can_execute(self) -> bool:
        return (
            self.state.can_execute(self.config.current_match_tolerance)
            and not self._stale_robot_names(self.state.selected_group_ids)
            and self._snapshots_cover_selected_robots(
                self.state.plan_state.robot_snapshots, self.state.selected_group_ids
            )
            and self.adapter.snapshots_match(
                self.state.plan_state.robot_snapshots, self.config.current_match_tolerance
            )
        )

    def _snapshots_cover_selected_robots(
        self, snapshots: Mapping[str, JointState], group_ids: tuple[PlanningGroupID, ...]
    ) -> bool:
        groups = self._groups_by_id()
        expected = {
            str(groups[group_id].robot_name) for group_id in group_ids if group_id in groups
        }
        return bool(expected) and expected == set(snapshots)

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
        target_pose = self.state.cartesian_target
        planned_target_joints = [
            float(position) for target in targets.values() for position in target.position
        ]
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                return
            self.state.action_status = ActionStatus.RUNNING
            self.state.plan_state.status = PlanStatus.PLANNING
            if self.state.manipulation_state == "FAULT":
                reset = self.adapter.reset()
                if not self._operation_is_current(
                    operation_id, selection_epoch, target_sequence_id
                ):
                    self._finish_operation(
                        "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                    )
                    return
                if not reset:
                    self.state.plan_state.status = PlanStatus.FAILED
                    self._finish_operation(
                        "reset=False",
                        clear_error=False,
                        operation_id=operation_id,
                        selection_epoch=selection_epoch,
                    )
                    return
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
            snapshots = self.adapter.snapshot_selected_robots(group_ids)
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            if snapshots is None:
                self.state.plan_state.status = PlanStatus.FAILED
                self.state.error = "Cannot plan without current states for all selected robots"
                self._finish_operation(
                    "plan=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            ok = self.adapter.plan_to_selected_joints(group_ids, targets)
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "plan=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            if ok:
                self.state.plan_state.status = PlanStatus.FRESH
                self.state.plan_state.group_ids = group_ids
                self.state.plan_state.target_sequence_id = target_sequence_id
                self.state.plan_state.robot_snapshots = snapshots
                self.state.plan_state.target_joints = planned_target_joints
                self.state.plan_state.target_pose = target_pose
            else:
                self.state.plan_state.status = PlanStatus.FAILED
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
        if self.scene is not None:
            self.scene.cancel_preview_animation()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch):
                return
            self.state.action_status = ActionStatus.PREVIEWING
            ok = self.adapter.preview_path()
            self._finish_operation(
                f"preview={ok}", operation_id=operation_id, selection_epoch=selection_epoch
            )

        self._operation_worker.submit(
            operation,
            timeout_seconds=self.config.preview_request_timeout,
            on_error=lambda message: self._set_operation_error(message, operation_id),
        )

    def _submit_execute(self) -> None:
        if self._closed:
            return
        if not self._can_execute():
            self._set_recoverable_error(
                "Cannot execute: require feasible fresh plan and matching current joints"
            )
            return
        selection_epoch = self.state.selection_epoch
        group_ids = self.state.selected_group_ids
        target_sequence_id = self.state.latest_sequence_id
        snapshots = self.state.plan_state.robot_snapshots
        expected_robot_names = tuple(self._selected_robot_names())
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                return
            if (
                self.state.plan_state.group_ids != group_ids
                or self.state.plan_state.target_sequence_id != target_sequence_id
                or not snapshots
                or set(snapshots) != set(expected_robot_names)
            ):
                self.state.plan_state.status = PlanStatus.STALE
                self._finish_operation(
                    "execute=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            stale_robots = self._stale_robot_names(group_ids)
            if stale_robots:
                if not self._operation_is_current(
                    operation_id, selection_epoch, target_sequence_id
                ):
                    self._finish_operation(
                        "execute=False",
                        operation_id=operation_id,
                        selection_epoch=selection_epoch,
                    )
                    return
                self.state.plan_state.status = PlanStatus.STALE
                self.state.error = "Cannot execute without fresh telemetry for: " + ", ".join(
                    stale_robots
                )
                self._finish_operation(
                    "execute=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            if not self.adapter.snapshots_match(snapshots, self.config.current_match_tolerance):
                if not self._operation_is_current(
                    operation_id, selection_epoch, target_sequence_id
                ):
                    self._finish_operation(
                        "execute=False",
                        operation_id=operation_id,
                        selection_epoch=selection_epoch,
                    )
                    return
                self.state.plan_state.status = PlanStatus.STALE
                self._finish_operation(
                    "execute=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            stale_robots = self._stale_robot_names(group_ids)
            if not self._operation_is_current(operation_id, selection_epoch, target_sequence_id):
                self._finish_operation(
                    "execute=False", operation_id=operation_id, selection_epoch=selection_epoch
                )
                return
            if stale_robots:
                self.state.plan_state.status = PlanStatus.STALE
                self.state.error = "Cannot execute without fresh telemetry for: " + ", ".join(
                    stale_robots
                )
                self._finish_operation(
                    "execute=False",
                    clear_error=False,
                    operation_id=operation_id,
                    selection_epoch=selection_epoch,
                )
                return
            self.state.action_status = ActionStatus.EXECUTING
            self.state.plan_state.status = PlanStatus.EXECUTING
            ok = self.adapter.execute()
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
        if self.scene is not None:
            self.scene.cancel_preview_animation()
        self._restart_operation_worker()
        try:
            ok = self.adapter.cancel()
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
        if self.scene is not None:
            self.scene.cancel_preview_animation()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.CLEARING_PLAN
            ok = self.adapter.clear_planned_path()
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
        self, result: TargetEvaluation, success: bool, collision_free: bool
    ) -> FeasibilityStatus:
        status = str(result.get("status", "")).upper()
        if success and collision_free:
            return FeasibilityStatus.FEASIBLE
        if status in {"COLLISION", "COLLISION_AT_START", "COLLISION_AT_GOAL"}:
            return FeasibilityStatus.COLLISION
        if status in {"NO_SOLUTION", "SINGULARITY", "JOINT_LIMITS", "TIMEOUT"}:
            return FeasibilityStatus.IK_FAILED
        return FeasibilityStatus.INVALID
