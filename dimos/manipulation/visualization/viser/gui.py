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
from math import degrees, radians
from typing import TypeAlias

import numpy as np

from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.manipulation.visualization.types import RobotInfo, TargetEvaluation
from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.reachability import (
    ReachabilityMapLayer,
    body_point_cloud,
    body_voxel_mesh,
    score_colors,
    slice_image_height,
    slice_image_yaw,
    vertical_slice_wxyz,
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
    | GuiSliderHandle[float]
    | TransformControlsHandle
)

# Fallback joint-slider range (radians) when a robot config omits joint limits.
DEFAULT_JOINT_LIMITS = (-3.14, 3.14)


class ViserPanelGui:
    """Optional operator panel with parity for the original cc/viser-vis panel."""

    def __init__(
        self,
        server: ViserServer,
        adapter: InProcessViserAdapter,
        config: ViserVisualizationConfig,
        scene: ViserManipulationScene | None = None,
        *,
        reachability_maps: Mapping[str, CapabilityMap] | None = None,
        reachability_layer: ReachabilityMapLayer | None = None,
    ) -> None:
        self.server = server
        self.adapter = adapter
        self.config = config
        self.scene = scene
        self.reachability_maps = dict(reachability_maps or {})
        self.reachability_layer = reachability_layer
        self.state = PanelState(runtime=PanelRuntime.STARTING)
        self._closed = False
        self._operation_sequence_id = 0
        self._suppress_target_callbacks = False
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

    def _build(self) -> None:
        gui = self.server.gui
        folder = gui.add_folder("Manipulation Panel", expand_by_default=True)
        self._handles["panel_folder"] = folder
        with folder:
            self._build_panel_controls(gui)

    def _build_panel_controls(self, gui: GuiApi) -> None:
        self._handles["status"] = gui.add_markdown("Starting manipulation panel...")
        # Live IK verdict: streams "solving..." with the current error during
        # the search, then the final reached/FAILED verdict with position and
        # rotation error magnitudes.
        self._handles["ik_status"] = gui.add_markdown("IK: idle")
        robots = self.adapter.list_robots()
        self._build_scene_controls(gui)
        self._build_reachability_controls(gui)
        robot_dropdown = gui.add_dropdown(
            "Robot",
            options=robots or [""],
            initial_value=robots[0] if robots else "",
        )
        robot_dropdown.on_update(lambda event: self._select_robot(event.target.value))
        self._handles["robot"] = robot_dropdown
        preset_dropdown = gui.add_dropdown(
            "Target Preset",
            options=["Select preset...", "Current"],
            initial_value="Select preset...",
        )
        preset_dropdown.on_update(lambda event: self._apply_preset(event.target.value))
        self._handles["preset"] = preset_dropdown
        plan_button = gui.add_button("Plan", disabled=True)
        plan_button.on_click(lambda _: self._submit_plan())
        self._handles["plan"] = plan_button
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

    def _build_reachability_controls(self, gui: GuiApi) -> None:
        if self.reachability_layer is None or not self.reachability_maps:
            return
        first_map = next(iter(self.reachability_maps.values()))
        z_mid = (first_map.params.z_min + first_map.params.z_max) / 2.0
        folder = gui.add_folder("Reachability", expand_by_default=False)
        self._handles["reachability_folder"] = folder
        with folder:
            visible = gui.add_checkbox("Show reachability", initial_value=False)
            self._handles["reachability_visible"] = visible
            style = gui.add_dropdown(
                "Style",
                options=["points", "voxels"],
                initial_value="points",
            )
            self._handles["reachability_style"] = style
            dexterity = gui.add_slider(
                "Min dexterity [%]",
                min=0,
                max=60,
                step=1,
                initial_value=0,
            )
            self._handles["reachability_dexterity"] = dexterity
            point_size = gui.add_slider(
                "Point size [mm]",
                min=0,
                max=60,
                step=1,
                initial_value=5,
            )
            self._handles["reachability_point_size"] = point_size
            vertical_slice = gui.add_checkbox("Vertical slice", initial_value=False)
            self._handles["reachability_vertical_slice"] = vertical_slice
            yaw = gui.add_slider(
                "Slice yaw [deg]",
                min=-180,
                max=180,
                step=5,
                initial_value=0,
            )
            self._handles["reachability_slice_yaw"] = yaw
            horizontal_slice = gui.add_checkbox("Horizontal slice", initial_value=False)
            self._handles["reachability_horizontal_slice"] = horizontal_slice
            z = gui.add_slider(
                "Slice height [m]",
                min=first_map.params.z_min,
                max=first_map.params.z_max,
                step=first_map.params.cell,
                initial_value=z_mid,
            )
            self._handles["reachability_slice_z"] = z
        for handle in (
            visible,
            style,
            dexterity,
            point_size,
            vertical_slice,
            yaw,
            horizontal_slice,
            z,
        ):
            handle.on_update(lambda _event: self._refresh_reachability())
        self._sync_reachability_z_slider(first_map)

    def _refresh_reachability(self) -> None:
        if self._closed or self.reachability_layer is None:
            return
        cap = self._selected_reachability_map()
        if cap is None or not self._handle_bool("reachability_visible", False):
            self.reachability_layer.close()
            return

        self._sync_reachability_z_slider(cap)
        dexterity = self._handle_float("reachability_dexterity", 0.0) / 100.0
        if self._handle_str("reachability_style", "points") == "voxels":
            mesh, _n_voxels = body_voxel_mesh(cap, dexterity)
            self.reachability_layer.show_voxel_mesh(mesh)
        else:
            points, scores = body_point_cloud(cap, dexterity)
            colors = score_colors(scores, vmax=max(float(scores.max(initial=0.0)), 1e-9))
            point_size = self._handle_float("reachability_point_size", 5.0) / 1000.0
            self.reachability_layer.show_points(points, colors, point_size=point_size)

        if self._handle_bool("reachability_vertical_slice", False):
            yaw = self._handle_float("reachability_slice_yaw", 0.0)
            image, width, height = slice_image_yaw(cap, yaw)
            self.reachability_layer.show_vertical_slice(
                image,
                width=width,
                height=height,
                center_z=(cap.params.z_min + cap.params.z_max) / 2.0,
                wxyz=vertical_slice_wxyz(radians(yaw)),
            )
        else:
            self.reachability_layer.clear_vertical_slice()

        if self._handle_bool("reachability_horizontal_slice", False):
            z = self._handle_float(
                "reachability_slice_z",
                (cap.params.z_min + cap.params.z_max) / 2.0,
            )
            image, width, height = slice_image_height(cap, z)
            self.reachability_layer.show_horizontal_slice(
                image,
                width=width,
                height=height,
                z=z,
            )
        else:
            self.reachability_layer.clear_horizontal_slice()

    def _sync_reachability_z_slider(self, cap: CapabilityMap) -> None:
        handle = self._handles.get("reachability_slice_z")
        if handle is None:
            return
        params = cap.params
        self._set_optional_handle_attr(handle, "min", params.z_min)
        self._set_optional_handle_attr(handle, "max", params.z_max)
        self._set_optional_handle_attr(handle, "step", params.cell)
        current = self._handle_float(
            "reachability_slice_z",
            (params.z_min + params.z_max) / 2.0,
        )
        self._set_optional_handle_attr(
            handle, "value", float(np.clip(current, params.z_min, params.z_max))
        )

    def _selected_reachability_map(self) -> CapabilityMap | None:
        if self.state.selected_robot is None:
            return None
        return self.reachability_maps.get(self.state.selected_robot)

    def _handle_bool(self, key: str, default: bool) -> bool:
        return bool(getattr(self._handles.get(key), "value", default))

    def _handle_float(self, key: str, default: float) -> float:
        try:
            return float(getattr(self._handles.get(key), "value", default))
        except (TypeError, ValueError):
            return default

    def _handle_str(self, key: str, default: str) -> str:
        value = getattr(self._handles.get(key), "value", default)
        return value if isinstance(value, str) else default

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
        if self.scene is None or self.state.selected_robot is None:
            return
        robot_id = self.adapter.robot_id_for_name(self.state.selected_robot)
        if robot_id is None:
            return
        ee_control = self.scene.ensure_target_controls(str(robot_id), self._on_transform_update)
        if ee_control is not None:
            self._handles["ee_control"] = ee_control
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
        if self.state.selected_robot is None:
            return
        gui = self.server.gui
        config = self.adapter.get_robot_config(self.state.selected_robot)
        if config is None:
            return
        current = self.adapter.get_current_joint_state(self.state.selected_robot)
        values = list(current.position) if current is not None else [0.0] * len(config.joint_names)
        self._clear_joint_sliders()
        joint_limits_lower = config.joint_limits_lower
        joint_limits_upper = config.joint_limits_upper
        for index, joint_name in enumerate(config.joint_names):
            lower, upper = DEFAULT_JOINT_LIMITS
            if joint_limits_lower is not None and index < len(joint_limits_lower):
                lower = joint_limits_lower[index]
            if joint_limits_upper is not None and index < len(joint_limits_upper):
                upper = joint_limits_upper[index]
            handle = gui.add_slider(
                joint_name,
                min=float(lower),
                max=float(upper),
                step=0.001,
                initial_value=float(values[index] if index < len(values) else 0.0),
            )

            def on_update(_event: object, name: str = joint_name) -> None:
                self._on_joint_slider_update(name)

            handle.on_update(on_update)
            self._joint_sliders[joint_name] = handle

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
        self._refresh_reachability()

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
        if handle is None or self.state.selected_robot is None:
            return
        info: RobotInfo | None = self.adapter.get_robot_info(self.state.selected_robot)
        config = self.adapter.get_robot_config(self.state.selected_robot)
        options = ["Select preset..."]
        if (info is not None and info["init_joints"] is not None) or self.adapter.get_init_joints(
            self.state.selected_robot
        ) is not None:
            options.append("Init")
        options.append("Current")
        home_joints = config.home_joints if config is not None else None
        if (info is not None and info["home_joints"] is not None) or home_joints is not None:
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
            values = list(config.home_joints or [])
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
        if self._closed:
            return
        if self._suppress_target_callbacks:
            return
        self._submit_joint_target_evaluation()

    def _on_transform_update(self, target: TransformControlsHandle) -> None:
        if self._closed:
            return
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
        # No refresh() here: a gizmo drag fires tens of events per second and
        # this callback runs in the client's event thread, so a full panel
        # resync per event queues later gizmo updates behind it and the gizmo
        # freezes on weak CPUs. _apply_target_evaluation_result refreshes once
        # per coalesced evaluation instead.

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
    ) -> TargetEvaluation:
        if request.source == "cartesian":
            if request.pose is None:
                return {"success": False, "status": "INVALID", "message": "No pose target"}
            return self.adapter.evaluate_pose_target(
                request.pose, request.robot_name, on_step=self._ik_step_animator(request)
            )
        if request.joints is None:
            return {"success": False, "status": "INVALID", "message": "No joint target"}
        return self.adapter.evaluate_joint_target(request.joints, request.robot_name)

    def _ik_step_animator(
        self, request: TargetEvaluationRequest
    ) -> Callable[[JointState, float, int], bool]:
        """Animate the IK search on the target ghost, aborting stale searches.

        Runs on the evaluation worker thread. The ghost shows the solver's
        current guess flowing toward the gizmo instead of teleporting to the
        final answer; if the user drags again mid-search, the sequence ID
        advances and the stale solve aborts so the new target starts sooner.
        """
        robot_id = self.adapter.robot_id_for_name(request.robot_name)

        def on_step(guess: JointState, position_error: float, attempt: int) -> bool:
            if self._closed or request.sequence_id != self.state.latest_sequence_id:
                return True
            if self.scene is not None and robot_id is not None:
                self.scene.set_target_joints(str(robot_id), guess.name, list(guess.position))
            self._set_handle_value(
                "ik_status",
                f"IK: solving... attempt {attempt + 1} | pos {position_error * 1000:.0f} mm",
            )
            return False

        return on_step

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: TargetEvaluation
    ) -> None:
        if self._closed:
            return
        if request.sequence_id != self.state.latest_sequence_id:
            return
        collision_free = bool(result.get("collision_free", False))
        success = bool(result.get("success", False))
        self.state.feasibility.status = self._feasibility_status(result, success, collision_free)
        self.state.feasibility.message = str(result.get("message", ""))
        if request.source == "cartesian":
            self._set_handle_value("ik_status", self._ik_verdict_text(result, collision_free))
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
            pose = result.get("ee_pose")
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
        # Do not write the Cartesian target back into the active transform
        # control here. The gizmo is the source of truth for Cartesian edits;
        # programmatic pose writes from delayed IK results can fight fast user
        # dragging and make the gizmo jump back.

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
            not self.state.can_execute(self.config.current_match_tolerance),
        )
        self._set_disabled("cancel", not self.state.can_cancel())
        self._update_target_visual_state()

    def _update_target_visual_state(self) -> None:
        if self.scene is None or self.state.selected_robot is None:
            return
        robot_id = self.adapter.robot_id_for_name(self.state.selected_robot)
        if robot_id is None:
            return
        self.scene.set_target_visual_state(
            str(robot_id), self.state.feasibility.status == FeasibilityStatus.FEASIBLE
        )

    def _submit_plan(self) -> None:
        if self._closed:
            return
        robot_name = self.state.selected_robot
        if robot_name is None:
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
            if self.state.manipulation_state == "FAULT" and not self.adapter.reset():
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation("reset=False", clear_error=False, operation_id=operation_id)
                return
            target = self._target_from_sliders(robot_name)
            if target is None:
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation(
                    "plan_to_joints=False", clear_error=False, operation_id=operation_id
                )
                return
            ok = self.adapter.plan_to_joints(target, robot_name)
            if not self._operation_is_current(operation_id):
                return
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
            self._finish_operation(f"plan_to_joints={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_preview(self) -> None:
        if self._closed:
            return
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        if not self.state.can_preview():
            self._set_recoverable_error("No fresh plan to preview")
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.PREVIEWING
            ok = self.adapter.preview_path(robot_name)
            self._finish_operation(f"preview={ok}", operation_id=operation_id)

        # The preview blocks for the whole ghost animation, and frame pushes
        # can lag on slow links -- a flat timeout shorter than the animation
        # bricks the panel (FAILED) on every long plan. Scale it with the
        # planned duration.
        planned_duration = self.adapter.get_planned_trajectory_duration(robot_name) or 3.0
        self._operation_worker.submit(
            operation,
            timeout_seconds=max(self.config.preview_request_timeout, planned_duration * 2 + 5.0),
            on_error=lambda message: self._set_operation_error(message, operation_id),
        )

    def _submit_execute(self) -> None:
        if self._closed:
            return
        robot_name = self.state.selected_robot
        if robot_name is None:
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
            ok = self.adapter.execute(robot_name)
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
        robot_name = self.state.selected_robot
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.CLEARING_PLAN
            ok = self.adapter.clear_planned_path(robot_name)
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
            # Also log: the panel error line is easy to miss and the UI may
            # not be visible when this fires.
            logger.warning(f"Viser panel operation failed: {message}")
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
            # viser >= 1.0 markdown handles expose ``content``; older ones
            # ``value``. Setting a missing attr is silently dropped by the
            # hasattr guard, which left the whole panel's text frozen at
            # "Starting manipulation panel..." on current viser.
            attr = "content" if hasattr(handle, "content") else "value"
            self._set_optional_handle_attr(handle, attr, value)

    def _set_disabled(self, key: str, disabled: bool) -> None:
        handle = self._handles.get(key)
        if isinstance(handle, GuiButtonHandle):
            self._set_optional_handle_attr(handle, "disabled", disabled)

    @staticmethod
    def _set_optional_handle_attr(handle: object, attr: str, value: object) -> None:
        setattr(handle, attr, value)

    def _pose_from_transform_target(self, target: TransformControlsHandle) -> Pose | None:
        px, py, pz = (float(value) for value in target.position)
        qw, qx, qy, qz = (float(value) for value in target.wxyz)
        return Pose({"position": [px, py, pz], "orientation": [qx, qy, qz, qw]})

    @staticmethod
    def _ik_verdict_text(result: TargetEvaluation, collision_free: bool) -> str:
        """Final IK verdict with error magnitudes, like the original viewer.

        Distinguishes "close but self-colliding" from "did not reach" so a
        red ghost is diagnosable at a glance.
        """
        status = str(result.get("status", "")).upper()
        reached = status in {"SUCCESS", "COLLISION"}
        verdict = "reached" if reached else "FAILED"
        if not collision_free:
            verdict += ", SELF-COLLISION"
        pos = result.get("position_error")
        rot = result.get("orientation_error")
        errors = []
        if isinstance(pos, (int, float)):
            errors.append(f"pos {float(pos) * 1000:.0f} mm")
        if isinstance(rot, (int, float)):
            errors.append(f"rot {degrees(float(rot)):.0f} deg")
        suffix = f" ({', '.join(errors)})" if errors else ""
        return f"IK {verdict}{suffix}"

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
