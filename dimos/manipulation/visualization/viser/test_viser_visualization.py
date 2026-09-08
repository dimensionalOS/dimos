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

"""Hermetic Viser panel, scene, obstacle, and preview behavior tests."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import trimesh

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    Obstacle,
    PlanningSceneInfo,
    VisualizationSession,
    VisualizationStateFrame,
)
from dimos.manipulation.visualization.operator import (
    ManipulationOperator,
    OperatorStatus,
    TargetEvaluationResult,
)
from dimos.manipulation.visualization.viser import (
    scene as scene_module,
    visualizer as visualizer_module,
)
from dimos.manipulation.visualization.viser.animation import (
    PreviewAnimation,
    PreviewFrame,
    preview_tick_times,
    scaled_frame_delays,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import PLANNING_MODE_LABELS, ViserPanelGui
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    PanelPlanState,
    PlanningMode,
    PlanStatus,
    TargetEvaluationRequest,
    TargetStatus,
)
from dimos.manipulation.visualization.viser.theme import apply_dimos_theme
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel


def _model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/model.urdf")),
        joint_names=["left/j1", "right/j1"],
        planning_groups=[
            PlanningGroupDefinition("left_arm", ("left/j1",), "base", "left/tool"),
            PlanningGroupDefinition("right_arm", ("right/j1",), "base", "right/tool"),
        ],
    )


def test_preview_timing_uses_one_model_track() -> None:
    preview = PreviewAnimation(
        ("left/j1", "right/j1"),
        (
            PreviewFrame(0.0, (0.0, 0.0)),
            PreviewFrame(1.0, (1.0, 0.5)),
            PreviewFrame(3.0, (2.0, 1.0)),
        ),
    )
    assert preview_tick_times(preview) == (0.0, 1.0, 3.0)
    assert scaled_frame_delays(preview.frames, 6.0) == (2.0, 4.0)


def test_visualizer_builds_full_model_preview_from_selected_canonical_joints() -> None:
    visualizer = ViserManipulationVisualizer()
    visualizer._model_config = _model()
    visualizer._current_state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    trajectory = JointTrajectory(
        joint_names=["right/j1"],
        points=[TrajectoryPoint(positions=[0.8], time_from_start=1.0)],
    )
    preview = visualizer._raw_preview_animation(trajectory)
    assert preview == PreviewAnimation(("left/j1", "right/j1"), (PreviewFrame(1.0, (0.1, 0.8)),))


def test_visualizer_rejects_unknown_or_duplicate_trajectory_joints() -> None:
    visualizer = ViserManipulationVisualizer()
    visualizer._model_config = _model()
    visualizer._current_state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    for names in (["unknown"], ["left/j1", "left/j1"]):
        trajectory = JointTrajectory(
            joint_names=names,
            points=[TrajectoryPoint(positions=[0.0] * len(names), time_from_start=1.0)],
        )
        assert visualizer._raw_preview_animation(trajectory) is None


def test_visualizer_initializes_and_updates_one_scene_model() -> None:
    visualizer = ViserManipulationVisualizer()
    scene = MagicMock()
    visualizer._scene = scene
    visualizer._runtime = MagicMock()
    visualizer._initialize_scene(PlanningSceneInfo(model=_model()))
    scene.register_model.assert_called_once()

    state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    visualizer.update_state(VisualizationStateFrame(joint_state=state))
    scene.update_current_model.assert_called_once_with(state)


@pytest.fixture
def scene_model(tmp_path):
    path = tmp_path / "model.urdf"
    path.write_text("""<robot name="arms"><link name="base"/>
<link name="left/tool"/><link name="right/tool"/>
<joint name="left/j1" type="revolute"><parent link="base"/><child link="left/tool"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
<joint name="right/j1" type="revolute"><parent link="base"/><child link="right/tool"/>
<axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>""")
    config = _model()
    config.model = RobotModel.from_file(path)
    config.base_link = "base"
    config.home_joints = [0.7, 0.8]
    config.joint_limits_lower = [-1.0, -2.0]
    config.joint_limits_upper = [1.0, 2.0]
    return config


@pytest.fixture
def scene(mocker):
    def handle(*args, **kwargs):
        return mocker.MagicMock(
            label=args[0] if args else "",
            value=kwargs.get("initial_value"),
            visible=kwargs.get("visible", True),
            disabled=kwargs.get("disabled", False),
            options=kwargs.get("options", []),
            show_visual=True,
            show_collision=False,
        )

    server = mocker.Mock()
    for name in (
        "add_folder",
        "add_markdown",
        "add_button",
        "add_dropdown",
        "add_checkbox",
        "add_slider",
    ):
        getattr(server.gui, name).side_effect = handle
    for name in (
        "add_grid",
        "add_transform_controls",
        "add_box",
        "add_icosphere",
        "add_cylinder",
        "add_label",
        "add_mesh_simple",
    ):
        getattr(server.scene, name).side_effect = handle
    value = ViserManipulationScene(server, mocker.Mock(side_effect=handle))
    try:
        yield value
    finally:
        value.close()


@pytest.fixture
def panel(scene, scene_model, mocker):
    scene.register_model(scene_model)
    current = mocker.Mock(
        return_value=JointState(name=scene_model.joint_names, position=[0.1, 0.2])
    )
    operator = mocker.Mock(spec=ManipulationOperator)
    operator.status.return_value = OperatorStatus("IDLE", "", False)
    operator.get_motion_speed.return_value = 1.0
    operator.get_init_joints.return_value = JointState(
        name=scene_model.joint_names, position=[-0.5, -1.0]
    )
    operator.evaluate_joint_target.side_effect = lambda request: TargetEvaluationResult(
        True,
        "FEASIBLE",
        "",
        True,
        group_ids=request.group_ids,
        target_joints=request.target,
        group_poses={name: PoseStamped(position=[0.1, 0.2, 0.3]) for name in request.group_ids},
    )
    groups = tuple(PlanningGroupRegistry(scene_model.planning_groups).list())
    gui = ViserPanelGui(
        scene.server,
        PlanningSceneInfo(model=scene_model, planning_groups=groups),
        operator,
        current,
        ViserVisualizationConfig(),
        scene,
    )
    mocker.patch.object(gui._worker, "start")
    mocker.patch.object(gui._worker, "submit")
    mocker.patch.object(gui._operation_worker, "start")
    mocker.patch.object(
        gui._operation_worker, "submit", side_effect=lambda operation, **kwargs: operation()
    )
    try:
        gui.start()
        yield gui, operator, current
    finally:
        gui.close()


def _obstacle(kind=ObstacleType.BOX, dimensions=(1.0, 2.0, 3.0), **kwargs):
    return Obstacle(
        name="shape",
        obstacle_type=kind,
        dimensions=dimensions,
        pose=PoseStamped(position=[1.0, 2.0, 3.0], orientation=[0.1, 0.2, 0.3, 0.4]),
        color=kwargs.pop("color", (0.2, 0.4, 0.6, 0.75)),
        **kwargs,
    )


@pytest.mark.parametrize(
    ("kind", "dimensions", "method", "geometry"),
    [
        (ObstacleType.BOX, (1.0, 2.0, 3.0), "add_box", {"dimensions": (1.0, 2.0, 3.0)}),
        (ObstacleType.SPHERE, (0.4,), "add_icosphere", {"radius": 0.4}),
        (ObstacleType.CYLINDER, (0.5, 1.5), "add_cylinder", {"radius": 0.5, "height": 1.5}),
    ],
)
def test_scene_renders_obstacle_geometry_pose_and_color(scene, kind, dimensions, method, geometry):
    scene.add_vis_obstacle("shape", _obstacle(kind, dimensions))
    getattr(scene.server.scene, method).assert_called_once_with(
        "/manipulation/obstacles/shape",
        **geometry,
        color=(51, 102, 153),
        opacity=0.75,
        position=(1.0, 2.0, 3.0),
        wxyz=(0.4, 0.1, 0.2, 0.3),
        visible=True,
    )


@pytest.mark.parametrize(
    "color", [(0.1, 0.2, 0.3), (float("nan"), 0.2, 0.3, 0.4), (0.1, 0.2, 0.3, 1.5)]
)
def test_scene_invalid_color_uses_fallback(scene, color):
    scene.add_vis_obstacle("shape", _obstacle(color=color))
    kwargs = scene.server.scene.add_box.call_args.kwargs
    assert (kwargs["color"], kwargs["opacity"]) == ((55, 190, 210), 0.55)


def test_scene_invalid_geometry_has_visible_proxy_and_reason(scene):
    scene.add_vis_obstacle("shape", _obstacle(dimensions=(1.0,)))
    assert scene.server.scene.add_box.call_args.args[0].endswith("mesh-failure-proxy")
    assert scene.server.scene.add_box.call_args.kwargs["visible"] is True
    assert "box dimensions" in str(scene.server.scene.add_label.call_args)


def test_scene_mesh_loading_and_failure_proxy(scene, mocker):
    mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], faces=[[0, 1, 2]])
    load = mocker.patch.object(scene_module.trimesh, "load_mesh", return_value=trimesh.Scene(mesh))
    scene.add_vis_obstacle("mesh", _obstacle(ObstacleType.MESH, (), mesh_path="triangle.obj"))
    args = scene.server.scene.add_mesh_simple.call_args
    assert args.args[0] == "/manipulation/obstacles/mesh"
    np.testing.assert_array_equal(args.args[1], mesh.vertices)
    np.testing.assert_array_equal(args.args[2], mesh.faces)
    load.side_effect = OSError("missing")
    scene.add_vis_obstacle("missing", _obstacle(ObstacleType.MESH, (), mesh_path="missing.obj"))
    assert (
        scene.server.scene.add_box.call_args.args[0]
        == "/manipulation/obstacles/missing/mesh-failure-proxy"
    )
    assert (
        scene.server.scene.add_label.call_args.args[0]
        == "/manipulation/obstacles/missing/mesh-failure-label"
    )


def test_scene_obstacle_replacement_visibility_and_cleanup(scene):
    item = _obstacle()
    scene.add_vis_obstacle("shape", item)
    original = scene._obstacle_handles["shape"][0]
    scene.set_obstacles_visible(False)
    assert original.visible is False
    scene.add_vis_obstacle("shape", item)
    original.remove.assert_called_once()
    replacement = scene._obstacle_handles["shape"][0]
    assert replacement.visible is False
    scene.clear_vis_obstacles()
    replacement.remove.assert_called_once()
    assert scene._obstacles == {}
    scene.close()
    count = scene.server.scene.add_box.call_count
    scene.add_vis_obstacle("closed", item)
    scene.set_obstacles_visible(True)
    assert scene.server.scene.add_box.call_count == count


def test_scene_pose_update_preserves_geometry_and_replaces_handle(scene):
    scene.add_vis_obstacle("shape", _obstacle())
    original = scene._obstacle_handles["shape"][0]
    scene.update_vis_obstacle_pose("shape", PoseStamped(position=[7.0, 8.0, 9.0]))
    original.remove.assert_called_once()
    kwargs = scene.server.scene.add_box.call_args.kwargs
    assert kwargs["dimensions"] == (1.0, 2.0, 3.0)
    assert kwargs["position"] == (7.0, 8.0, 9.0)
    assert scene._obstacles["shape"].dimensions == (1.0, 2.0, 3.0)
    scene.update_vis_obstacle_pose("missing", PoseStamped())
    assert scene.server.scene.add_box.call_count == 2


def test_scene_update_reports_proxy_failure_and_reuses_warning(scene, mocker):
    mocker.patch.object(scene_module.trimesh, "load_mesh", side_effect=OSError("missing"))
    with pytest.raises(RuntimeError, match="renderer used a proxy"):
        scene.update_vis_obstacle(_obstacle(ObstacleType.MESH, (), mesh_path="missing.obj"))
    scene.show_obstacle_warning("first")
    handle = scene._obstacle_warning_handle
    scene.show_obstacle_warning("second")
    assert scene._obstacle_warning_handle is handle
    assert handle.content == "second"


@pytest.fixture
def mock_visualizer(mocker):
    runtime = mocker.patch.object(visualizer_module, "ViserRuntime").return_value
    renderer = mocker.patch.object(visualizer_module, "ViserManipulationScene").return_value
    visualizer = ViserManipulationVisualizer(config=ViserVisualizationConfig(panel_enabled=False))
    try:
        visualizer.initialize(VisualizationSession(PlanningSceneInfo(model=_model())))
        yield visualizer, renderer, runtime
    finally:
        visualizer.close()


def test_visualizer_forwards_obstacle_operations_until_closed(mock_visualizer):
    visualizer, renderer, runtime = mock_visualizer
    item = _obstacle()
    visualizer.add_vis_obstacle(item.name, item)
    visualizer.update_vis_obstacle(item)
    visualizer.update_vis_obstacle_pose(item.name, item.pose)
    visualizer.remove_vis_obstacle(item.name)
    visualizer.clear_vis_obstacles()
    renderer.add_vis_obstacle.assert_called_once_with(item.name, item)
    renderer.update_vis_obstacle.assert_called_once_with(item)
    renderer.update_vis_obstacle_pose.assert_called_once_with(item.name, item.pose)
    renderer.remove_vis_obstacle.assert_called_once_with(item.name)
    renderer.clear_vis_obstacles.assert_called_once_with()
    visualizer.close()
    renderer.close.assert_called_once_with()
    runtime.close.assert_called_once_with()
    renderer.reset_mock()
    visualizer.add_vis_obstacle(item.name, item)
    visualizer.update_vis_obstacle(item)
    visualizer.update_vis_obstacle_pose(item.name, item.pose)
    visualizer.remove_vis_obstacle(item.name)
    visualizer.clear_vis_obstacles()
    assert renderer.mock_calls == []


@pytest.mark.parametrize("method", ["update_vis_obstacle", "update_vis_obstacle_pose"])
def test_visualizer_update_failure_is_reported_once(mock_visualizer, method):
    visualizer, renderer, _ = mock_visualizer
    getattr(renderer, method).side_effect = RuntimeError("renderer failed")
    item = _obstacle()
    args = (item,) if method == "update_vis_obstacle" else (item.name, item.pose)
    getattr(visualizer, method)(*args)
    getattr(renderer, method).assert_called_once_with(*args)
    renderer.show_obstacle_warning.assert_called_once()
    assert "shape" in renderer.show_obstacle_warning.call_args.args[0]
    assert "renderer failed" in renderer.show_obstacle_warning.call_args.args[0]


def test_panel_selects_pose_group_and_exposes_controls(panel):
    gui, _, _ = panel
    assert gui.state.selected_group_ids == ("left_arm",)
    assert gui._handles["preset"].options == ["Select preset...", "Init", "Current", "Home"]
    assert {
        "plan",
        "preview",
        "execute",
        "cancel",
        "clear",
        "next_plan_speed",
    } <= gui._handles.keys()
    assert set(gui._joint_sliders) == {("left_arm", "left/j1")}
    assert gui.state.group_joint_targets["left_arm"].position == [0.1]


@pytest.mark.parametrize(
    ("preset", "expected"), [("Init", [-0.5, -1.0]), ("Current", [0.1, 0.2]), ("Home", [0.7, 0.8])]
)
def test_panel_presets_update_selected_canonical_joints(panel, preset, expected):
    gui, _, _ = panel
    gui._toggle_group_selected("right_arm")
    gui._apply_preset(preset)
    assert gui.state.group_joint_targets["left_arm"].position == expected[:1]
    assert gui.state.group_joint_targets["right_arm"].position == expected[1:]
    assert [
        gui._joint_sliders[(group, joint)].value
        for group, joint in (("left_arm", "left/j1"), ("right_arm", "right/j1"))
    ] == expected


def test_panel_incomplete_preset_does_not_partially_change_targets(panel):
    gui, operator, _ = panel
    gui._toggle_group_selected("right_arm")
    operator.get_init_joints.return_value = JointState(name=["left/j1"], position=[0.9])
    before = {key: list(value.position) for key, value in gui.state.group_joint_targets.items()}
    gui._apply_preset("Init")
    assert {
        key: list(value.position) for key, value in gui.state.group_joint_targets.items()
    } == before
    assert "missing joints for right_arm" in gui.state.error


def test_panel_initialization_waits_for_complete_telemetry(panel):
    gui, _, current = panel
    gui.state.group_joint_targets.clear()
    gui._clear_joint_sliders()
    current.return_value = JointState(name=["right/j1"], position=[0.2])
    gui.refresh()
    assert gui.state.group_joint_targets == {}
    assert gui._joint_sliders == {}
    current.return_value = JointState(name=["left/j1", "right/j1"], position=[0.3, 0.4])
    gui.refresh()
    assert gui.state.group_joint_targets["left_arm"].position == [0.3]
    assert gui._joint_sliders[("left_arm", "left/j1")].value == 0.3


def test_panel_current_preset_requires_telemetry(panel):
    gui, _, current = panel
    current.return_value = None
    before = list(gui.state.group_joint_targets["left_arm"].position)
    gui._apply_preset("Current")
    assert "without fresh telemetry" in gui.state.error
    assert gui.state.group_joint_targets["left_arm"].position == before


def test_panel_speed_change_preserves_current_plan(panel):
    gui, operator, _ = panel
    gui.state.plan_state = PanelPlanState(status=PlanStatus.FRESH)
    plan_state = gui.state.plan_state
    sequence = gui.state.latest_sequence_id
    gui._set_next_plan_speed(0.5)
    operator.set_motion_speed.assert_called_once_with(0.5)
    assert gui.state.plan_state is plan_state
    assert gui.state.plan_state.status == PlanStatus.FRESH
    assert gui.state.latest_sequence_id == sequence


@pytest.mark.parametrize("action", [ActionStatus.RUNNING, ActionStatus.PREVIEWING])
def test_panel_speed_disabled_during_operations(panel, action):
    gui, operator, _ = panel
    gui.state.action_status = action
    gui.refresh()
    assert gui._handles["next_plan_speed"].disabled is True
    gui._set_next_plan_speed(0.5)
    operator.set_motion_speed.assert_not_called()


@pytest.mark.parametrize("success", [True, False])
def test_panel_cartesian_plan_uses_requested_mode_without_fallback(panel, success):
    gui, operator, _ = panel
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.planning_mode = PlanningMode.CARTESIAN_SPACE
    plan = GeneratedPlan(
        group_ids=("left_arm",),
        status=PlanningStatus.SUCCESS,
        trajectory=JointTrajectory(
            joint_names=["left/j1"], points=[TrajectoryPoint(positions=[0.1], time_from_start=0.0)]
        ),
    )
    operator.plan_cartesian.return_value = plan if success else None
    operator.status.return_value = OperatorStatus("IDLE", "" if success else "IK failed", False)
    gui._submit_plan()
    operator.plan_cartesian.assert_called_once()
    request = operator.plan_cartesian.call_args.args[0]
    assert tuple(request.pose_targets) == ("left_arm",)
    assert request.pose_targets["left_arm"].frame_id == "world"
    assert (request.config.speed_mode, request.config.dt) == ("time_optimal", 0.05)
    operator.plan_to_joints.assert_not_called()
    assert gui.state.plan_state.status == (PlanStatus.FRESH if success else PlanStatus.FAILED)
    assert gui.state.error == ("" if success else "IK failed")


def test_panel_mode_change_invalidates_accepted_plan(panel):
    gui, _, _ = panel
    gui.state.plan_state = PanelPlanState(status=PlanStatus.FRESH)
    gui._set_planning_mode(PLANNING_MODE_LABELS[PlanningMode.CARTESIAN_SPACE])
    assert gui.state.plan_state.status == PlanStatus.STALE


def test_transform_callback_preserves_pose_through_backend_request(panel):
    gui, operator, _ = panel
    control = gui.scene._handles["left_arm:ee_control"]
    control.position = (0.7, 0.8, 0.9)
    control.wxyz = (0.4, 0.1, 0.2, 0.3)
    callback = control.on_update.call_args.args[0]
    callback(SimpleNamespace(target=control))
    request = gui._worker.submit.call_args.args[0]
    gui._handle_target_evaluation_request(request)
    dispatched = operator.evaluate_pose_target.call_args.args[0].pose_targets["left_arm"]
    assert list(dispatched.position) == [0.7, 0.8, 0.9]
    assert list(dispatched.orientation) == [0.1, 0.2, 0.3, 0.4]


def test_joint_evaluation_updates_gizmo_and_rejects_stale_result(panel):
    gui, _, _ = panel
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=gui.state.selected_group_ids,
    )
    pose = PoseStamped(position=[0.7, 0.8, 0.9], orientation=[0.1, 0.2, 0.3, 0.4])
    result = TargetEvaluationResult(True, "FEASIBLE", "", True, group_poses={"left_arm": pose})
    gui._apply_target_evaluation_result(request, result)
    control = gui.scene._handles["left_arm:ee_control"]
    assert control.position == (0.7, 0.8, 0.9)
    assert control.wxyz == (0.4, 0.1, 0.2, 0.3)
    gui.state.next_sequence_id()
    gui.state.target_status = TargetStatus.CHECKING
    gui._apply_target_evaluation_result(request, result)
    assert gui.state.target_status == TargetStatus.CHECKING


def test_scene_target_tracks_current_until_explicit_target(scene, scene_model):
    scene.register_model(scene_model)
    scene.set_target_active(True)
    scene.update_current_model(JointState(name=scene_model.joint_names, position=[0.1, 0.2]))
    target = scene._urdfs["target"]
    target.update_cfg.assert_called_with([0.1, 0.2])
    assert scene.set_target_joints(scene_model.joint_names, [0.7, 0.8]) is True
    scene.update_current_model(JointState(name=scene_model.joint_names, position=[0.3, 0.4]))
    target.update_cfg.assert_called_with([0.7, 0.8])
    scene.clear_target()
    scene.update_current_model(JointState(name=scene_model.joint_names, position=[0.3, 0.4]))
    target.update_cfg.assert_called_with([0.3, 0.4])


@pytest.mark.parametrize("replace", [False, True])
def test_scene_inflight_preview_stops_after_cancellation_or_replacement(
    scene, scene_model, mocker, replace
):
    scene.register_model(scene_model)
    initial = PreviewAnimation(
        tuple(scene_model.joint_names),
        (PreviewFrame(0.0, (0.1, 0.2)), PreviewFrame(1.0, (0.3, 0.4))),
    )
    replacement = PreviewAnimation(tuple(scene_model.joint_names), (PreviewFrame(0.0, (0.7, 0.8)),))

    def interrupt(_delay):
        scene.cancel_preview_animation()
        if replace:
            assert scene.animate_preview(replacement, 0.0) is True

    mocker.patch.object(scene_module.time, "sleep", side_effect=interrupt)
    assert scene.animate_preview(initial, 1.0) is False
    updates = [call.args[0] for call in scene._urdfs["preview"].update_cfg.call_args_list]
    assert updates == ([[0.1, 0.2], [0.7, 0.8]] if replace else [[0.1, 0.2]])
    assert scene._preview_visible is False


def test_scene_empty_preview_and_missing_target_are_rejected(scene):
    assert scene.animate_preview(PreviewAnimation((), ()), 0.0) is False
    assert scene.set_target_joints(["unknown"], [0.0]) is False


def test_theme_reference_grid_and_toggle(scene):
    assert apply_dimos_theme(scene.server) is True
    theme = scene.server.gui.configure_theme.call_args.kwargs
    assert (theme["brand_color"], theme["dark_mode"], theme["control_layout"]) == (
        (0, 153, 255),
        True,
        "fixed",
    )
    assert scene.has_reference_grid()
    grid = scene.server.scene.add_grid.call_args.kwargs
    assert (grid["width"], grid["height"], grid["plane"]) == (20.0, 20.0, "xy")
    scene.set_reference_grid_visible(False)
    assert scene._grid_handle.visible is False


@pytest.mark.parametrize("feasible", [True, False])
def test_panel_feasibility_colors_active_controls_and_single_ghost(panel, mocker, feasible):
    gui, _, _ = panel
    gui._toggle_group_selected("right_arm")
    ghost_mesh = mocker.Mock()
    gui.scene._urdfs["target"]._meshes = [ghost_mesh]
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=gui.state.selected_group_ids,
    )
    gui._apply_target_evaluation_result(
        request,
        TargetEvaluationResult(feasible, "FEASIBLE" if feasible else "COLLISION", "", feasible),
    )
    expected_control = (
        scene_module.TARGET_CONTROL_FEASIBLE_COLOR
        if feasible
        else scene_module.TARGET_CONTROL_INFEASIBLE_COLOR
    )
    expected_ghost = (
        scene_module.GOAL_ROBOT_FEASIBLE_COLOR
        if feasible
        else scene_module.GOAL_ROBOT_INFEASIBLE_COLOR
    )
    assert gui.scene._handles["left_arm:ee_control"].color == expected_control
    assert gui.scene._handles["right_arm:ee_control"].color == expected_control
    assert ghost_mesh.color == expected_ghost


def test_panel_plan_actions_require_feasible_target_and_fresh_plan(panel):
    gui, _, _ = panel
    gui.state.target_status = TargetStatus.INFEASIBLE
    gui.refresh()
    assert [gui._handles[key].disabled for key in ("plan", "preview", "execute")] == [
        True,
        True,
        True,
    ]
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.refresh()
    assert [gui._handles[key].disabled for key in ("plan", "preview", "execute")] == [
        False,
        True,
        True,
    ]


def test_panel_group_change_hides_inactive_control_and_preserves_source_labels(panel):
    gui, _, _ = panel
    assert gui._handles["group:left_arm"].label == "left_arm"
    gui._toggle_group_selected("right_arm")
    old_control = gui.scene._handles["left_arm:ee_control"]
    gui._toggle_group_selected("left_arm")
    assert gui.state.selected_group_ids == ("right_arm",)
    assert set(gui.scene._handles) == {"right_arm:ee_control"}
    old_control.remove.assert_called_once()
    assert gui.scene._target_active is True


def test_panel_home_preset_can_initialize_targets_without_complete_telemetry(panel):
    gui, _, current = panel
    current.return_value = None
    gui.state.group_joint_targets.clear()
    gui._clear_joint_sliders()
    gui._apply_preset("Home")
    assert gui.state.group_joint_targets["left_arm"].position == [0.7]
    assert gui._joint_sliders[("left_arm", "left/j1")].value == 0.7


def test_panel_joint_slider_limits_match_model(panel):
    gui, _, _ = panel
    gui._toggle_group_selected("right_arm")
    calls = {call.args[0]: call.kwargs for call in gui.server.gui.add_slider.call_args_list}
    assert (calls["left_arm/left/j1"]["min"], calls["left_arm/left/j1"]["max"]) == (-1.0, 1.0)
    assert (calls["right_arm/right/j1"]["min"], calls["right_arm/right/j1"]["max"]) == (-2.0, 2.0)


def test_visualizer_update_without_scene_does_not_start_network(mocker):
    visualizer = ViserManipulationVisualizer()
    start = mocker.patch.object(visualizer, "_ensure_started")
    try:
        visualizer.update_vis_obstacle(_obstacle())
        visualizer.update_vis_obstacle_pose("shape", PoseStamped())
        assert start.call_count == 2
        assert visualizer._scene is None
    finally:
        visualizer.close()
