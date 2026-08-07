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

from unittest.mock import MagicMock

from dimos.manipulation import pnpconsole
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray


def test_client_scans_scene_and_quits(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.scan_scene.return_value = MagicMock(detections_length=3)
    app = MagicMock(pnp=pnp)
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["1", "", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    pnp.scan_scene.assert_called_once_with(None)


def test_client_scans_scene_with_text_prompt(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.scan_scene.return_value = MagicMock(detections_length=1)
    app = MagicMock(pnp=pnp)
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["1", "water bottle", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    pnp.scan_scene.assert_called_once_with("water bottle")


def test_client_describes_current_scene(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.describe_scene.return_value = "A blue block is on the table."
    app = MagicMock(pnp=pnp)
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["16", "", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    pnp.describe_scene.assert_called_once_with("What objects are visible on the table?")


def test_client_does_not_execute_without_a_plan(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    app = MagicMock()
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["5", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.execute_and_wait.assert_not_called()


def test_client_goes_home(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    app = MagicMock()
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["13", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.go_home.assert_called_once_with("arm")


def test_client_does_not_execute_descent_without_a_plan(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    app = MagicMock()
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["7", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.execute_and_wait.assert_not_called()


def test_client_previews_descent_before_explicit_execution(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    goal = PoseStamped(position=Vector3(0.1, 0.2, 0.3))
    pre_grasp = PoseStamped(position=Vector3(0.1, 0.2, 0.2))
    pnp = MagicMock()
    pnp.get_goal_pose.return_value = goal
    pnp.get_grasp_candidates.return_value = GraspCandidateArray()
    pnp.get_pre_grasp_pose.return_value = pre_grasp
    app = MagicMock(pnp=pnp)
    manipulation = app.ManipulationModule
    manipulation.plan_to_pose.return_value = True
    manipulation.execute_and_wait.return_value = True
    manipulation.get_ee_pose.return_value = pre_grasp
    manipulation.plan_cartesian_targets.return_value = True
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["3", "1", "4", "5", "6", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    assert manipulation.execute_and_wait.call_count == 1
    manipulation.preview_plan.assert_called_with(duration=2.0)


def test_client_runs_grasp_and_lift_without_preview(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    goal = PoseStamped(position=Vector3(0.1, 0.2, 0.3))
    pre_grasp = PoseStamped(position=Vector3(0.1, 0.2, 0.2))
    pnp = MagicMock()
    pnp.get_goal_pose.return_value = goal
    pnp.get_grasp_candidates.return_value = GraspCandidateArray()
    pnp.get_pre_grasp_pose.return_value = pre_grasp
    app = MagicMock(pnp=pnp)
    manipulation = app.ManipulationModule
    manipulation.plan_to_pose.return_value = True
    manipulation.plan_cartesian_targets.return_value = True
    manipulation.execute_and_wait.return_value = True
    manipulation.get_ee_pose.return_value = pre_grasp
    manipulation.close_gripper.return_value.is_success.return_value = True
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    monkeypatch.setattr(pnpconsole.time, "sleep", lambda _: None)
    choices = iter(["3", "1", "4", "5", "15", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    assert manipulation.execute_and_wait.call_count == 3
    assert manipulation.plan_cartesian_targets.call_count == 2
    manipulation.close_gripper.assert_called_once_with("arm")
    manipulation.preview_plan.assert_called_once_with(duration=2.0)


def test_client_installs_table_collision_with_recommended_clearance(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.estimate_table_surface.return_value = {
        "center_x": 0.5,
        "center_y": 0.0,
        "tabletop_z": 0.35,
        "width": 0.8,
        "depth": 1.0,
    }
    app = MagicMock(pnp=pnp)
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["14", "", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    pnp.scan_scene.assert_called_once_with()
    manipulation.set_table_collision.assert_called_once_with(
        0.5, 0.0, 0.35, 0.8, 1.0, safety_margin=0.01
    )


def test_client_accepts_zero_table_clearance(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.estimate_table_surface.return_value = {
        "center_x": 0.5,
        "center_y": 0.0,
        "tabletop_z": 0.35,
        "width": 0.8,
        "depth": 1.0,
    }
    app = MagicMock(pnp=pnp)
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["14", "0", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.set_table_collision.assert_called_once_with(
        0.5, 0.0, 0.35, 0.8, 1.0, safety_margin=0.0
    )


def test_preview_plays_once_slowly() -> None:
    manipulation = MagicMock()

    pnpconsole._preview(manipulation)

    manipulation.preview_plan.assert_called_once_with(duration=2.0)


def test_grasp_rank_accepts_default_and_valid_selection(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    choices = iter(["", "7"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    assert pnpconsole._grasp_rank(10) == 0
    assert pnpconsole._grasp_rank(10) == 7
