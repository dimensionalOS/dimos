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

import time
from types import SimpleNamespace
from unittest.mock import Mock

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds
from dimos.simulation.habitat.connection import HabitatConnection, HabitatConnectionConfig


def environment(**kwargs):
    return HabitatEnvironment(blueprint=["habitat-nav", "mcp-server", "observe-skill"], **kwargs)


@pytest.mark.parametrize("missing", ["color_image", "habitat_scan", "odometry", None])
def test_smoke_checks_recorded_samples(mocker, missing):
    from dimos.evals.suites.habitat_smoke import sensor_score

    store = mocker.patch(
        "dimos.evals.suites.habitat_smoke.recording"
    ).return_value.__enter__.return_value
    store.streams.__contains__.side_effect = lambda name: name != missing
    assert sensor_score(SimpleNamespace(artifacts={})) == (1.0 if missing is None else 0.0)
    if missing is None:
        store.streams.habitat_scan.last.assert_called_once()


def test_scene_config_reaches_blueprint_parser(tmp_path):
    from dimos.simulation.habitat.blueprints import habitat_nav

    dataset = tmp_path / "dataset.json"
    dataset.write_text("{}")
    env = environment(
        scene_dataset_config=str(dataset),
        scene_id="apt_1",
        seed=42,
        start_position_ros_override=(1, 2, 3),
        start_yaw_deg=15,
    )
    proc = DimosCliCall()
    env.configure_launch(proc)
    parsed = BlueprintConfigParser(habitat_nav).parse(environ=proc.extra_env)
    config = parsed.module_kwargs("habitatconnection")
    assert config["scene_id"] == "apt_1"
    assert config["scene_dataset_config"] == str(dataset)
    assert config["start_position_ros"] == (1, 2, 3)
    assert "start_position_ros_override" not in config
    assert config["seed"] == 42
    assert config["publish_semantic"] is False
    assert config["source_dir"] == HabitatConnectionConfig().source_dir
    assert config["build_command"] == HabitatConnectionConfig().build_command
    assert parsed.global_config["transport"] == "zenoh"
    assert proc.simulator is None
    overrides = env.episode_metadata()["connection_overrides"]
    assert overrides["start_position_ros"] == [1, 2, 3]
    assert "start_position_ros_override" not in overrides


def test_invalid_configuration(tmp_path):
    with pytest.raises(ValueError, match="finite"):
        environment(start_position_ros_override=(float("nan"), 0, 0))
    with pytest.raises(ValueError, match="fresh launches"):
        environment(attach=True)
    with pytest.raises(FileNotFoundError):
        environment(scene_dataset_config=str(tmp_path / "missing")).preflight(Mock())


@pytest.mark.parametrize("relative", [False, True])
@pytest.mark.parametrize("scene", [None, "default", "custom"])
def test_external_executable_reaches_blueprint_without_source_build(
    tmp_path, monkeypatch, mocker, relative, scene
):
    executable = tmp_path / "habitat-native"
    executable.touch()
    dataset = (
        tmp_path
        / "data/versioned_data/hm3d-0.2/hm3d/example"
        / "hm3d_annotated_example_basis.scene_dataset_config.json"
    )
    dataset.parent.mkdir(parents=True)
    dataset.write_text("{}")
    custom_dataset = tmp_path / "custom.json"
    custom_dataset.write_text("{}")
    monkeypatch.chdir(tmp_path)
    source = mocker.patch(
        "dimos.core.native_module.get_project_root", side_effect=AssertionError("source fetch")
    )
    env = environment(
        executable=executable.name if relative else str(executable),
        scene_dataset_config="custom.json" if scene == "custom" else scene,
    )
    proc = DimosCliCall()

    env.configure_launch(proc)
    monkeypatch.chdir(tmp_path.parent)
    parsed = BlueprintConfigParser(HabitatConnection.blueprint()).parse(environ=proc.extra_env)
    config = HabitatConnectionConfig(**parsed.module_kwargs("habitatconnection"))

    assert config.resolve_paths() == (str(tmp_path), str(executable))
    assert config.source_dir is None
    assert config.build_command is None
    expected_scene = {None: str(dataset), "default": "default", "custom": str(custom_dataset)}
    assert config.scene_dataset_config == expected_scene[scene]
    source.assert_not_called()


def test_launch_and_cleanup(tmp_path, mocker):
    proc = mocker.patch("dimos.evals.environments.sim.DimosCliCall").return_value
    proc.extra_env = {}
    mocker.patch(
        "dimos.evals.environments.sim.McpAdapter"
    ).return_value.wait_for_ready.return_value = True
    store = mocker.patch("dimos.memory.store.sqlite.SqliteStore").return_value
    env = environment(scene_id="apt_1")
    mocker.patch.object(env, "_wait_recording", return_value=tmp_path / "memory.db")
    ready = mocker.patch.object(env, "wait_ready")
    try:
        result = env.start(("speak-skill",))
        assert proc.simulator is None
        assert proc.global_args[0] == "--record-topics"
        assert proc.global_args[-1] == "--record"
        from dimos.memory.tap import matching

        assert matching(
            proc.global_args[1],
            ("depth_image", "node_edges", "color_image", "habitat_scan", "odometry"),
        ) == {"color_image", "habitat_scan", "odometry"}
        assert proc.demo_args == [
            "run",
            "habitat-nav",
            "mcp-server",
            "observe-skill",
            "speak-skill",
        ]
        assert proc.extra_env["DIMOS_TRANSPORT"] == "zenoh"
        ready.assert_called_once()
        assert result.artifacts["episode"].is_file()
    finally:
        env.stop()
    proc.stop.assert_called_once()
    store.stop.assert_called_once()


def test_readiness_failure_releases_resources(tmp_path, mocker):
    proc = mocker.patch("dimos.evals.environments.sim.DimosCliCall").return_value
    mocker.patch(
        "dimos.evals.environments.sim.McpAdapter"
    ).return_value.wait_for_ready.return_value = True
    store = mocker.patch("dimos.memory.store.sqlite.SqliteStore").return_value
    env = environment()
    mocker.patch.object(env, "_wait_recording", return_value=tmp_path / "memory.db")
    mocker.patch.object(env, "wait_ready", side_effect=TimeoutError("no RGB"))
    try:
        with pytest.raises(TimeoutError, match="no RGB"):
            env.start(())
    finally:
        env.stop()
        env.stop()
    proc.stop.assert_called_once()
    store.stop.assert_called_once()


def test_readiness_and_pose_normalization():
    from dimos.memory.store.memory import MemoryStore

    env = environment()
    with MemoryStore() as store:
        with pytest.raises(LookupError):
            env.latest_pose(store)
        with pytest.raises(TimeoutError):
            env.wait_ready(store, deadline=time.monotonic())
        odom = Odometry(
            header=Header(frame_id="world", stamp=time_from_seconds(time.time())),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=1, y=2, z=3),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
            child_frame_id="",
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        )
        store.stream("odometry", Odometry).append(odom)
        store.stream("color_image", Image).append(
            image_from_array(
                np.zeros((2, 2, 3), dtype=np.uint8), encoding="rgb8", header=odom.header
            )
        )
        env.wait_ready(store, deadline=time.monotonic() + 1)
        pose = env.latest_pose(store)
        assert pose.header.stamp == odom.header.stamp
        assert pose.header.frame_id == "world"
        assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (1, 2, 3)
        assert env.episode_metadata()["initial_observed_position_ros"] == [1, 2, 3]


@pytest.mark.parametrize("navigable", [True, False])
def test_explicit_spawn_uses_real_constructor(mocker, navigable):
    from dimos.simulation.habitat.server import HabitatHost

    hs = Mock()
    mocker.patch.dict("sys.modules", {"habitat_sim": hs})
    sim = hs.Simulator.return_value
    sim.semantic_scene.objects = [None]  # an empty annotation slot keeps its index
    sim.pathfinder.is_navigable.return_value = navigable
    config = dict(
        scene_id="example",
        scene_dataset_config="default",
        seed=4,
        start_position_ros=(1, 2, 3),
        width=640,
        height=360,
        hfov_deg=90,
        camera_height_m=0.45,
    )
    if not navigable:
        with pytest.raises(ValueError, match="not navigable"):
            HabitatHost(config)
    else:
        host = HabitatHost(config)
        assert host.labels == [""]
        state = sim.initialize_agent.return_value.set_state.call_args.args[0]
        np.testing.assert_allclose(state.position, [-2, 3, -1])
        sim.pathfinder.get_random_navigable_point.assert_not_called()
        assert host.width == 640
