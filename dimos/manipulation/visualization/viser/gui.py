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

from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, TypeAlias, cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
from dimos.manipulation.visualization.types import (
    RobotInfo,
    TargetEvaluation,
    TargetSetEvaluation,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.panel_backend import (
    copy_joint_state,
    evaluate_joint_target_set,
    evaluate_pose_target_set,
    feasibility_status,
    get_current_joint_state,
    get_ee_pose,
    is_state_stale,
    joint_values_by_name,
    pose_from_transform_values,
    update_target_visual_state,
)
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


def group_display_name(group: PlanningGroup) -> str:
    robot_name = group.robot_name
    group_name = group.group_name
    return robot_name if group_name == "manipulator" else f"{robot_name} {group_name}"


def group_selector_color(
    selected: bool,
    active_color: tuple[int, int, int],
    inactive_color: tuple[int, int, int],
) -> tuple[int, int, int]:
    return active_color if selected else inactive_color


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
        self.world_monitor = world_monitor
        self.manipulation_module = manipulation_module
        self.config = config
        self.scene = scene
        self.state = PanelState(runtime=PanelRuntime.STARTING)
        self._closed = False
        self._operation_sequence_id = 0
        self._suppress_target_callbacks = False
        self._default_group_initialized = False
        self._handles: dict[str, PanelHandle] = {}
        self._joint_sliders: dict[str, GuiSliderHandle[float]] = {}
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
        self._worker.stop()
        self._operation_worker.stop(timeout=2.0)
        self._clear_joint_sliders()
        self._remove_panel_handles()
        self._handles.clear()
        self.state.runtime = PanelRuntime.STOPPED

    def refresh(self) -> None:
        if self._closed:
            return
        robots = self._list_robots()
        groups = self._list_planning_groups()
        self.state.backend_status = (
            BackendConnectionStatus.READY if robots else BackendConnectionStatus.WAITING_FOR_ROBOT
        )
        if not self.state.selected_group_ids and groups and not self._default_group_initialized:
            first_pose_group = next((group for group in groups if group.has_pose_target), groups[0])
            self.state.selected_group_ids = (first_pose_group.id,)
            self.state.target_status = TargetStatus.EMPTY
            self._default_group_initialized = True
            self._sync_group_selection_state()
            self._initialize_selected_group_targets()
            self._build_joint_sliders()
        self._sync_group_selector(groups)
        self._refresh_selected_robot_state()
        self._ensure_scene_controls()
        self._sync_target_ghost_visibility()
        self._sync_preset_dropdown()
        self._update_status_text()
        self._update_target_summary()
        self._update_control_state()

    def _list_robots(self) -> list[RobotName]:
        return list(self.manipulation_module.list_robots())

    def _list_planning_groups(self) -> list[PlanningGroup]:
        return self.manipulation_module.list_planning_groups()

    def _get_robot_info(self, robot_name: RobotName) -> RobotInfo | None:
        return self.manipulation_module.get_robot_info(robot_name)

    def _get_init_joints(self, robot_name: RobotName) -> JointState | None:
        return copy_joint_state(self.manipulation_module.get_init_joints(robot_name))

    def _get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        return get_current_joint_state(
            self.world_monitor,
            self.manipulation_module,
            robot_name,
        )

    def _is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        return is_state_stale(
            self.world_monitor,
            self.manipulation_module,
            robot_name,
            max_age,
        )

    def _get_ee_pose(
        self, robot_name: RobotName, joint_state: JointState | None = None
    ) -> Pose | None:
        return get_ee_pose(
            self.world_monitor,
            self.manipulation_module,
            self._list_planning_groups(),
            robot_name,
            joint_state,
        )

    def _get_module_state(self) -> str:
        return str(self.manipulation_module.get_state())

    def _reset(self) -> bool:
        result = self.manipulation_module.reset()
        return result if isinstance(result, bool) else result.is_success()

    def _evaluate_joint_target_set(
        self, joint_targets: dict[PlanningGroupID, JointState]
    ) -> TargetSetEvaluation:
        return evaluate_joint_target_set(self.manipulation_module, joint_targets)

    def _evaluate_pose_target_set(
        self,
        pose_targets: dict[PlanningGroupID, Pose],
        auxiliary_groups: tuple[PlanningGroupID, ...] = (),
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> TargetSetEvaluation:
        return evaluate_pose_target_set(
            self.manipulation_module,
            pose_targets,
            auxiliary_groups=auxiliary_groups,
            seed=seed,
            check_collision=check_collision,
        )

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
        self._sync_group_selector(self._list_planning_groups())
        self._handles["target_heading"] = gui.add_markdown("### Target")
        preset_dropdown = gui.add_dropdown(
            "Preset",
            options=["Select preset...", "Current"],
            initial_value="Select preset...",
        )
        preset_dropdown.on_update(lambda event: self._apply_preset(event.target.value))
        self._handles["preset"] = preset_dropdown
        self._handles["target_summary"] = gui.add_markdown(
            f"Feasibility: `{self.state.feasibility.status.value}`"
        )
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
            self.state.manipulation_state = self._get_module_state()
            return
        self.state.robot_info = self._get_robot_info(robot_name)
        current = self._get_current_joint_state(robot_name)
        self.state.current_joints = list(current.position) if current is not None else None
        self.state.current_ee_pose = self._get_ee_pose(robot_name)
        self.state.manipulation_state = self._get_module_state()
        adapter_error = self.manipulation_module.get_error()
        if adapter_error:
            self.state.error = adapter_error

    def _ensure_scene_controls(self) -> None:
        if self.scene is None:
            return
        groups = self._group_info_by_id()
        active_pose_groups = set(self._selected_pose_group_ids())
        for key in [key for key in self._handles if key.startswith("ee_control:")]:
            group_id = key.split(":", 1)[1]
            if group_id in active_pose_groups:
                continue
            handle = self._handles.pop(key)
            remove_target_controls = getattr(self.scene, "remove_target_controls", None)
            if callable(remove_target_controls):
                remove_target_controls(group_id)
            else:
                remove = getattr(handle, "remove", None)
                if callable(remove):
                    remove()
        for group_id in active_pose_groups:
            group = groups.get(group_id)
            if group is None or not bool(group.has_pose_target):
                continue
            handle_key = f"ee_control:{group_id}"
            if handle_key in self._handles:
                continue
            ee_control = self.scene.ensure_target_controls(
                group_id,
                cast(
                    "Callable[[TransformControlsHandle], None]",
                    lambda target, gid=group_id: self._on_transform_update(gid, target),
                ),
            )
            if ee_control is not None:
                self._handles[handle_key] = ee_control
            pose = self.state.pose_targets.get(group_id)
            if pose is not None:
                self._suppress_target_callbacks = True
                try:
                    self.scene.set_target_pose(group_id, pose)
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
        groups = self._group_info_by_id()
        target_by_name: dict[str, float] = {}
        if self.state.target_joints is not None:
            target_by_name.update(
                zip(self.state.target_joints.name, self.state.target_joints.position, strict=False)
            )
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            config = self.manipulation_module.get_robot_config(str(group.robot_name))
            current = self._get_current_joint_state(str(group.robot_name))
            current_by_name = joint_values_by_name(str(group.robot_name), current)
            joint_limits_lower = config.joint_limits_lower if config is not None else None
            joint_limits_upper = config.joint_limits_upper if config is not None else None
            for index, (global_name, local_name) in enumerate(
                zip(group.joint_names, group.local_joint_names, strict=False)
            ):
                joint_name = str(global_name)
                local = str(local_name)
                value = float(
                    target_by_name.get(
                        joint_name,
                        target_by_name.get(
                            local, current_by_name.get(joint_name, current_by_name.get(local, 0.0))
                        ),
                    )
                )
                lower, upper = DEFAULT_JOINT_LIMITS
                if joint_limits_lower is not None and index < len(joint_limits_lower):
                    lower = joint_limits_lower[index]
                if joint_limits_upper is not None and index < len(joint_limits_upper):
                    upper = joint_limits_upper[index]
                handle = gui.add_slider(
                    f"{group_id}/{local}",
                    min=float(lower),
                    max=float(upper),
                    step=0.001,
                    initial_value=value,
                )

                def on_update(_event: object, name: str = joint_name) -> None:
                    self._on_joint_slider_update(name)

                handle.on_update(on_update)
                self._joint_sliders[joint_name] = handle

    def _target_set_from_sliders(self) -> dict[PlanningGroupID, JointState] | None:
        groups = self._group_info_by_id()
        targets: dict[PlanningGroupID, JointState] = {}
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                self._set_error(f"Unknown planning group: {group_id}")
                return None
            names = [str(name) for name in group.joint_names]
            positions: list[float] = []
            for name in names:
                handle = self._joint_sliders.get(name)
                if handle is None:
                    self._set_error(f"Missing target slider for {name}")
                    return None
                positions.append(float(handle.value))
            targets[group_id] = JointState({"name": names, "position": positions})
        return targets

    def _split_target_joints_by_group(self, target_joints: JointState) -> None:
        groups = self._group_info_by_id()
        positions_by_name = dict(zip(target_joints.name, target_joints.position, strict=False))
        self.state.group_joint_targets.clear()
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            names = [str(name) for name in group.joint_names]
            if not all(name in positions_by_name for name in names):
                continue
            self.state.group_joint_targets[group_id] = JointState(
                {"name": names, "position": [float(positions_by_name[name]) for name in names]}
            )

    def _clear_joint_sliders(self) -> None:
        for handle in self._joint_sliders.values():
            try:
                handle.remove()
            except AttributeError:
                pass
        self._joint_sliders.clear()

    def _remove_panel_handles(self) -> None:
        for key, handle in list(self._handles.items()):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
            self._handles.pop(key, None)

    def _sync_group_selector(self, groups: list[PlanningGroup]) -> None:
        seen_keys: set[str] = set()
        selected = set(self.state.selected_group_ids)
        for group in sorted(
            groups, key=lambda item: (not bool(item.has_pose_target), str(item.id))
        ):
            group_id = str(group.id)
            key = f"group:{group_id}"
            seen_keys.add(key)
            handle = self._handles.get(key)
            is_selected = group_id in selected
            label = group_display_name(group)
            if handle is None:
                handle = self.server.gui.add_button(
                    label,
                    color=group_selector_color(
                        is_selected, ACTIVE_GROUP_COLOR, INACTIVE_GROUP_COLOR
                    ),
                    hint="Click to toggle this planning group in the target set.",
                )
                handle.on_click(
                    cast(
                        "Callable[[object], None]",
                        lambda _event, gid=group_id: self._toggle_group_selected(gid),
                    )
                )
                self._handles[key] = handle
            else:
                self._set_optional_handle_attr(handle, "label", label)
                self._set_optional_handle_attr(
                    handle,
                    "color",
                    group_selector_color(is_selected, ACTIVE_GROUP_COLOR, INACTIVE_GROUP_COLOR),
                )

        for key in [key for key in self._handles if key.startswith("group:")]:
            if key not in seen_keys:
                handle = self._handles.pop(key)
                remove = getattr(handle, "remove", None)
                if callable(remove):
                    remove()

    def _set_group_selected(self, group_id: PlanningGroupID, selected: bool) -> None:
        current = list(self.state.selected_group_ids)
        if selected and group_id not in current:
            current.append(group_id)
        elif not selected and group_id in current:
            current.remove(group_id)
        self.state.selected_group_ids = tuple(current)
        self._sync_group_selection_state()
        self._prune_inactive_group_state()
        self._initialize_selected_group_targets()
        self.state.mark_plan_stale()
        self._build_joint_sliders()
        self.refresh()

    def _toggle_group_selected(self, group_id: PlanningGroupID) -> None:
        self._set_group_selected(group_id, group_id not in self.state.selected_group_ids)

    def _select_all_manipulators(self) -> None:
        groups = self._list_planning_groups()
        manipulator_groups = [
            str(group.id) for group in groups if str(group.group_name) == "manipulator"
        ]
        self.state.selected_group_ids = tuple(
            manipulator_groups or [str(group.id) for group in groups]
        )
        self._sync_group_selection_state()
        self._initialize_selected_group_targets()
        self._build_joint_sliders()
        self.refresh()

    def _clear_group_selection(self) -> None:
        if self._closed:
            return
        self.state.selected_group_ids = ()
        self._sync_group_selection_state()
        self._prune_inactive_group_state()
        self.state.target_status = TargetStatus.EMPTY
        self.state.feasibility.status = FeasibilityStatus.UNKNOWN
        self.state.plan_state = PanelPlanState()
        self._build_joint_sliders()
        self.refresh()

    def _group_info_by_id(self) -> dict[PlanningGroupID, PlanningGroup]:
        return {str(group.id): group for group in self._list_planning_groups()}

    def _sync_selected_robot_from_groups(self) -> None:
        groups = self._group_info_by_id()
        first_group = (
            groups.get(self.state.selected_group_ids[0]) if self.state.selected_group_ids else None
        )
        self.state.selected_robot = None if first_group is None else str(first_group.robot_name)

    def _sync_group_selection_state(self) -> None:
        self._sync_selected_robot_from_groups()
        self.state.auxiliary_group_ids = self._selected_auxiliary_group_ids()

    def _selected_pose_group_ids(self) -> tuple[PlanningGroupID, ...]:
        groups = self._group_info_by_id()
        return tuple(
            group_id
            for group_id in self.state.selected_group_ids
            if (group := groups.get(group_id)) is not None and bool(group.has_pose_target)
        )

    def _selected_auxiliary_group_ids(self) -> tuple[PlanningGroupID, ...]:
        groups = self._group_info_by_id()
        return tuple(
            group_id
            for group_id in self.state.selected_group_ids
            if (group := groups.get(group_id)) is not None and not bool(group.has_pose_target)
        )

    def _active_pose_targets(self) -> dict[PlanningGroupID, Pose]:
        return {
            group_id: self.state.pose_targets[group_id]
            for group_id in self._selected_pose_group_ids()
            if group_id in self.state.pose_targets
        }

    def _prune_inactive_group_state(self) -> None:
        selected = set(self.state.selected_group_ids)
        for mapping in (
            self.state.pose_targets,
            self.state.group_joint_targets,
            self.state.group_poses,
            self.state.group_diagnostics,
        ):
            for group_id in [group_id for group_id in mapping if group_id not in selected]:
                mapping.pop(group_id, None)
        self._refresh_target_joints_from_groups()

    def _initialize_selected_group_targets(self) -> None:
        groups = self._group_info_by_id()
        for group_id in self.state.selected_group_ids:
            if group_id in self.state.group_joint_targets:
                continue
            group = groups.get(group_id)
            if group is None:
                continue
            current = self._get_current_joint_state(str(group.robot_name))
            if current is None:
                continue
            current_by_name = joint_values_by_name(str(group.robot_name), current)
            names = [str(name) for name in group.joint_names]
            local_names = [str(name) for name in group.local_joint_names]
            positions = [
                float(current_by_name.get(global_name, current_by_name.get(local, 0.0)))
                for global_name, local in zip(names, local_names, strict=False)
            ]
            self.state.group_joint_targets[group_id] = JointState(
                {"name": names, "position": positions}
            )
            if bool(group.has_pose_target) and group_id not in self.state.pose_targets:
                pose = self._get_ee_pose(str(group.robot_name))
                if pose is not None:
                    self.state.pose_targets[group_id] = pose
                    self.state.group_poses[group_id] = pose
                    if self.state.cartesian_target is None:
                        self.state.cartesian_target = pose
        self._refresh_target_joints_from_groups()

    def _refresh_target_joints_from_groups(self) -> None:
        names: list[str] = []
        positions: list[float] = []
        for group_id in self.state.selected_group_ids:
            target = self.state.group_joint_targets.get(group_id)
            if target is None:
                continue
            names.extend(target.name)
            positions.extend(target.position)
        self.state.target_joints = (
            JointState({"name": names, "position": positions}) if names else None
        )

    def _current_snapshot_by_group(self) -> dict[PlanningGroupID, list[float]]:
        groups = self._group_info_by_id()
        snapshot: dict[PlanningGroupID, list[float]] = {}
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            current = self._get_current_joint_state(str(group.robot_name))
            if current is None:
                continue
            current_by_name = joint_values_by_name(str(group.robot_name), current)
            snapshot[group_id] = [
                float(
                    current_by_name.get(str(global_name), current_by_name.get(str(local_name), 0.0))
                )
                for global_name, local_name in zip(
                    group.joint_names, group.local_joint_names, strict=False
                )
            ]
        return snapshot

    def _sync_preset_dropdown(self) -> None:
        handle = self._handles.get("preset")
        if handle is None:
            return
        selected_robot_names = self._selected_robot_names()
        options = ["Select preset..."]
        if any(
            self._get_init_joints(robot_name) is not None for robot_name in selected_robot_names
        ):
            options.append("Init")
        options.append("Current")
        if any(
            (config := self.manipulation_module.get_robot_config(robot_name)) is not None
            and config.home_joints is not None
            for robot_name in selected_robot_names
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
        groups = [
            group
            for group in self._list_planning_groups()
            if group.id in self.state.selected_group_ids
        ]
        for group in groups:
            robot_name = str(group.robot_name)
            values_by_name = self._preset_values_by_name(preset, robot_name)
            global_names = [str(name) for name in group.joint_names]
            local_names = [str(name) for name in group.local_joint_names]
            values = [
                float(values_by_name.get(local_name, values_by_name.get(global_name, 0.0)))
                for local_name, global_name in zip(local_names, global_names, strict=False)
            ]
            self._set_slider_values(global_names, values)
        self.state.joint_target = [float(handle.value) for handle in self._joint_sliders.values()]
        self._submit_joint_target_evaluation()
        self.refresh()

    def _selected_robot_names(self) -> tuple[str, ...]:
        groups = self._group_info_by_id()
        names: list[str] = []
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            robot_name = str(group.robot_name)
            if robot_name not in names:
                names.append(robot_name)
        return tuple(names)

    def _preset_values_by_name(self, preset: str, robot_name: str) -> dict[str, float]:
        if preset == "Current":
            current = self._get_current_joint_state(robot_name)
            if current is None:
                return {}
            return {
                str(name): float(value)
                for name, value in zip(current.name, current.position, strict=False)
            }
        if preset == "Init":
            init = self._get_init_joints(robot_name)
            if init is None:
                return {}
            return {
                str(name): float(value)
                for name, value in zip(init.name, init.position, strict=False)
            }
        config = self.manipulation_module.get_robot_config(robot_name)
        if config is None:
            return {}
        return {
            str(name): float(value)
            for name, value in zip(config.joint_names, config.home_joints or [], strict=False)
        }

    def _set_slider_values(self, joint_names: list[str], values: list[float]) -> None:
        self._suppress_target_callbacks = True
        try:
            for joint_name, value in zip(joint_names, values, strict=False):
                handle = self._joint_sliders.get(joint_name)
                if handle is not None:
                    handle.value = float(value)
        finally:
            self._suppress_target_callbacks = False

    def _on_joint_slider_update(self, _joint_name: str) -> None:
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
        pose = pose_from_transform_values(target.position.tolist(), target.wxyz.tolist())
        self.state.cartesian_target = pose
        self.state.pose_targets[group_id] = pose
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="cartesian",
                group_ids=self.state.selected_group_ids,
                auxiliary_group_ids=self._selected_auxiliary_group_ids(),
                pose_targets=self._active_pose_targets(),
                check_collision=True,
            )
        )
        self.refresh()

    def _submit_joint_target_evaluation(self) -> None:
        targets = self._target_set_from_sliders()
        if targets is None:
            return
        self.state.group_joint_targets = targets
        self._refresh_target_joints_from_groups()
        self._move_joint_target_visuals()
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="joints",
                group_ids=self.state.selected_group_ids,
                joint_targets=dict(targets),
            )
        )
        self.refresh()

    def _move_joint_target_visuals(self) -> None:
        """Optimistically move target visuals before collision/feasibility returns."""
        if self.scene is None:
            return
        groups = self._group_info_by_id()
        for group_id, target in self.state.group_joint_targets.items():
            group = groups.get(group_id)
            if group is None:
                continue
            robot_name = str(group.robot_name)
            robot_id = self.manipulation_module.robot_id_for_name(robot_name)
            config = self.manipulation_module.get_robot_config(robot_name)
            if robot_id is None or config is None:
                continue
            local_positions = dict(zip(target.name, target.position, strict=False))
            joints = [
                float(local_positions.get(str(global_name), 0.0))
                for global_name in group.joint_names
            ]
            self.scene.set_target_joints(str(robot_id), list(group.local_joint_names), joints)

    def _sync_target_ghost_visibility(self) -> None:
        if self.scene is None:
            return
        active_robot_ids: set[str] = set()
        groups = self._group_info_by_id()
        for group_id in self._selected_pose_group_ids():
            group = groups.get(group_id)
            if group is None:
                continue
            robot_id = self.manipulation_module.robot_id_for_name(str(group.robot_name))
            if robot_id is not None:
                active_robot_ids.add(str(robot_id))
        set_target_active = getattr(self.scene, "set_target_active", None)
        if not callable(set_target_active):
            return
        for _robot_name, robot_id, _config in self.manipulation_module.robot_items():
            set_target_active(str(robot_id), str(robot_id) in active_robot_ids)

    def _handle_target_evaluation_request(
        self, request: TargetEvaluationRequest
    ) -> TargetEvaluation | TargetSetEvaluation:
        if request.source == "cartesian":
            if not request.pose_targets:
                return {"success": False, "status": "INVALID", "message": "No pose target"}
            return self._evaluate_pose_target_set(
                request.pose_targets,
                auxiliary_groups=request.auxiliary_group_ids,
                seed=self.state.last_valid_target_joints,
                check_collision=request.check_collision,
            )
        if not request.joint_targets:
            return {"success": False, "status": "INVALID", "message": "No joint target"}
        return self._evaluate_joint_target_set(request.joint_targets)

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: TargetEvaluation | TargetSetEvaluation
    ) -> None:
        if self._closed:
            return
        if request.sequence_id != self.state.latest_sequence_id:
            return
        collision_free = bool(result.get("collision_free", False))
        success = bool(result.get("success", False))
        self.state.feasibility.status = feasibility_status(
            str(result.get("status", "")), success, collision_free
        )
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
        group_poses = result.get("group_poses", {})
        if isinstance(group_poses, dict):
            self.state.group_poses = {
                str(group_id): pose
                for group_id, pose in group_poses.items()
                if isinstance(pose, Pose)
            }
        if request.source == "joints" and isinstance(target_joints, JointState):
            self._sync_pose_targets_from_group_poses()
        group_diagnostics = result.get("group_diagnostics", {})
        if isinstance(group_diagnostics, dict):
            self.state.group_diagnostics = {
                str(group_id): str(message) for group_id, message in group_diagnostics.items()
            }
        if request.source == "cartesian" and isinstance(target_joints, JointState):
            self._sync_controls_from_targets()
        self._update_target_visual_state()
        self.refresh()

    def _sync_controls_from_targets(self) -> None:
        if self.state.target_joints is not None:
            positions_by_name = dict(
                zip(self.state.target_joints.name, self.state.target_joints.position, strict=False)
            )
            self._set_slider_values(list(positions_by_name), list(positions_by_name.values()))
            self._move_joint_target_visuals()
        # Do not write the Cartesian target back into the active transform
        # control here. The gizmo is the source of truth for Cartesian edits;
        # programmatic pose writes from delayed IK results can fight fast user
        # dragging and make the gizmo jump back.

    def _sync_pose_targets_from_group_poses(self) -> None:
        groups = self._group_info_by_id()
        updated_group_ids: list[PlanningGroupID] = []
        for group_id, pose in self.state.group_poses.items():
            group = groups.get(group_id)
            if group is None or not bool(group.has_pose_target):
                continue
            if group_id not in self._selected_pose_group_ids():
                continue
            self.state.pose_targets[group_id] = pose
            updated_group_ids.append(group_id)
        first_group_id = next(iter(self._selected_pose_group_ids()), None)
        if first_group_id is not None:
            self.state.cartesian_target = self.state.pose_targets.get(first_group_id)
        self._sync_scene_target_pose_controls(updated_group_ids)

    def _sync_scene_target_pose_controls(self, group_ids: list[PlanningGroupID]) -> None:
        if self.scene is None:
            return
        self._suppress_target_callbacks = True
        try:
            for group_id in group_ids:
                pose = self.state.pose_targets.get(group_id)
                if pose is not None:
                    self.scene.set_target_pose(group_id, pose)
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
        if self.state.selected_robot is not None:
            status.append(f"State stale: `{self._is_state_stale(self.state.selected_robot)}`")
        if current is not None:
            status.append(f"Current joints: `{[round(v, 3) for v in current]}`")
        if self.state.last_result:
            status.append(f"Last result: `{self.state.last_result}`")
        self._set_handle_value("status", "\n\n".join(status))

    def _update_target_summary(self) -> None:
        self._set_handle_value(
            "target_summary", f"Feasibility: `{self.state.feasibility.status.value}`"
        )

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
        can_cancel = self.state.can_cancel()
        self._set_disabled("cancel", not can_cancel)
        self._set_visible("cancel", can_cancel)
        self._update_target_visual_state()

    def _update_target_visual_state(self) -> None:
        if self.scene is None:
            return
        update_target_visual_state(
            self.scene,
            self._group_info_by_id(),
            self.state.selected_group_ids,
            self.manipulation_module.robot_id_for_name,
            self.state.feasibility.status == FeasibilityStatus.FEASIBLE,
        )

    def _submit_plan(self) -> None:
        if self._closed:
            return
        if not self.state.selected_group_ids:
            self._set_recoverable_error(
                "Cannot plan until target is feasible and manipulation is idle"
            )
            return
        if not self.state.can_plan():
            self._set_recoverable_error(
                "Cannot plan until target is feasible and manipulation is idle"
            )
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.RUNNING
            self.state.plan_state.status = PlanStatus.PLANNING
            if self.state.manipulation_state == "FAULT" and not self._reset():
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation("reset=False", clear_error=False, operation_id=operation_id)
                return
            targets = self._target_set_from_sliders()
            if targets is None:
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation(
                    "plan_to_joints=False", clear_error=False, operation_id=operation_id
                )
                return
            ok = self.manipulation_module.plan_to_joint_targets(
                cast("Mapping[PlanningGroupID | PlanningGroup, JointState]", targets)
            )
            if not self._operation_is_current(operation_id):
                return
            if ok:
                self.state.plan_state.status = PlanStatus.FRESH
                self.state.plan_state.group_ids = self.state.selected_group_ids
                self.state.plan_state.robot = self.state.selected_robot
                self.state.plan_state.target_joints = list(
                    self.state.target_joints.position if self.state.target_joints else []
                )
                self.state.plan_state.target_pose = self.state.cartesian_target
                self.state.plan_state.start_joints_snapshot = self._current_snapshot_by_group()
                self.state.plan_state.planned_path = None
            else:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"plan_to_joints={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_preview(self) -> None:
        if self._closed:
            return
        if not self.state.can_preview():
            self._set_recoverable_error("No fresh plan to preview")
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.PREVIEWING
            ok = self.manipulation_module.preview_plan()
            self._finish_operation(f"preview={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation,
            timeout_seconds=self.config.preview_request_timeout,
            on_error=lambda message: self._set_operation_error(message, operation_id),
        )

    def _submit_execute(self) -> None:
        if self._closed:
            return
        if not self.config.allow_plan_execute:
            self._set_recoverable_error(
                "Panel execution disabled; set allow_plan_execute=True to enable"
            )
            return
        if not self.state.can_execute(self.config.current_match_tolerance):
            self._set_recoverable_error(
                "Cannot execute: require feasible fresh plan and matching current joints"
            )
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.EXECUTING
            self.state.plan_state.status = PlanStatus.EXECUTING
            ok = self.manipulation_module.execute()
            if not self._operation_is_current(operation_id):
                return
            if not ok:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"execute={ok}", operation_id=operation_id)

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
            ok = self.manipulation_module.cancel()
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
            ok = self.manipulation_module.clear_planned_path()
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

    def _operation_is_current(self, operation_id: int) -> bool:
        return not self._closed and operation_id == self._operation_sequence_id

    def _finish_operation(
        self, result: str, *, clear_error: bool = True, operation_id: int | None = None
    ) -> None:
        if self._closed or (
            operation_id is not None and not self._operation_is_current(operation_id)
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
        if handle is None:
            return
        if hasattr(handle, "content") or hasattr(handle, "value"):
            attr = "content" if hasattr(handle, "content") else "value"
            self._set_optional_handle_attr(handle, attr, value)

    def _set_disabled(self, key: str, disabled: bool) -> None:
        handle = self._handles.get(key)
        if handle is not None and hasattr(handle, "disabled"):
            self._set_optional_handle_attr(handle, "disabled", disabled)

    def _set_visible(self, key: str, visible: bool) -> None:
        handle = self._handles.get(key)
        if handle is not None:
            self._set_optional_handle_attr(handle, "visible", visible)

    @staticmethod
    def _set_optional_handle_attr(handle: object, attr: str, value: object) -> None:
        setattr(handle, attr, value)
