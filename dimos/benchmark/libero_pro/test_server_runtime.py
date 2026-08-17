"""Threading regression tests for the container-owned LIBERO runtime."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import threading

import numpy as np
from pytest_mock import MockerFixture

SERVER_PATH = Path(__file__).parents[3] / "docker" / "libero-pro" / "server.py"
SERVER_SPEC = spec_from_file_location("libero_pro_container_server", SERVER_PATH)
assert SERVER_SPEC is not None
assert SERVER_SPEC.loader is not None
SERVER = module_from_spec(SERVER_SPEC)
SERVER_SPEC.loader.exec_module(SERVER)


def test_environment_is_initialized_and_stepped_on_the_same_thread(
    mocker: MockerFixture,
) -> None:
    simulator_thread_ids: list[int] = []
    observation = {
        "robot0_joint_pos": np.zeros(7),
        "robot0_joint_vel": np.zeros(7),
        "robot0_gripper_qpos": np.zeros(2),
        "agentview_image": np.zeros((128, 128, 3), dtype=np.uint8),
        "agentview_depth": np.ones((128, 128, 1), dtype=np.float32),
        "robot0_eye_in_hand_image": np.zeros((128, 128, 3), dtype=np.uint8),
        "robot0_eye_in_hand_depth": np.ones((128, 128, 1), dtype=np.float32),
    }

    class Environment:
        def step(self, _action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, dict]:
            simulator_thread_ids.append(threading.get_ident())
            return observation, 1.0, True, {"success": True}

        def check_success(self) -> bool:
            return True

        def close(self) -> None:
            simulator_thread_ids.append(threading.get_ident())

    def initialize(runtime, _request) -> tuple[str, str]:
        simulator_thread_ids.append(threading.get_ident())
        runtime.environment = Environment()
        runtime.environment.sim = mocker.sentinel.sim
        runtime.get_real_depth_map = lambda _sim, depth: depth
        runtime.get_camera_to_robot_base = lambda _sim, _camera: np.eye(4)
        runtime.camera_intrinsics = {
            "agentview": np.eye(3),
            "robot0_eye_in_hand": np.eye(3),
        }
        runtime.frequency = 1_000
        runtime.horizon = 1
        runtime.target = np.zeros(8)
        runtime._action = lambda: np.zeros(7)
        runtime._publish(observation, 0)
        return "task", "instruction"

    mocker.patch.object(
        SERVER.Runtime, "_initialize_environment", autospec=True, side_effect=initialize
    )
    runtime = SERVER.Runtime()
    try:
        ready = runtime.initialize(object())
        runtime.start()
        result = runtime.wait_result()
    finally:
        runtime.stop()

    assert ready == ("task", "instruction")
    assert result.success
    assert len(simulator_thread_ids) == 3
    assert len(set(simulator_thread_ids)) == 1
    assert simulator_thread_ids[0] != threading.get_ident()


def test_camera_pose_is_expressed_in_the_robot_base_frame() -> None:
    class Model:
        @staticmethod
        def body_name2id(name: str) -> int:
            assert name == "robot0_base"
            return 0

    class Data:
        xpos = np.array([[0.4, -0.2, 0.5]])
        xmat = np.array([np.eye(3).reshape(-1)])

    class Simulator:
        model = Model()
        data = Data()

    camera_to_world = np.eye(4)
    camera_to_world[:3, 3] = [0.6, 0.1, 1.2]

    camera_to_robot_base = SERVER._camera_to_robot_base(Simulator(), camera_to_world)

    assert np.allclose(camera_to_robot_base[:3, 3], [0.2, 0.3, 0.7])


def test_camera_rendering_is_cached_outside_the_control_step() -> None:
    calls: list[str] = []

    class Simulator:
        @staticmethod
        def render(*, width: int, height: int, camera_name: str, depth: bool):
            assert (width, height, depth) == (128, 128, True)
            calls.append(camera_name)
            return (
                np.full((128, 128, 3), len(calls), dtype=np.uint8),
                np.full((128, 128), len(calls), dtype=np.float32),
            )

    runtime = object.__new__(SERVER.Runtime)
    runtime.environment = type("Environment", (), {"sim": Simulator()})()

    runtime._render_cameras()

    assert calls == ["agentview", "robot0_eye_in_hand"]
    assert runtime.camera_observation["agentview_image"][0, 0, 0] == 1
    assert runtime.camera_observation["robot0_eye_in_hand_depth"][0, 0] == 2
    assert SERVER.CAMERA_RENDER_INTERVAL_TICKS == 4


def test_joint_target_is_adapted_to_native_osc_pose_action() -> None:
    runtime = object.__new__(SERVER.Runtime)
    runtime.snapshot = type(
        "Snapshot",
        (),
        {"gripper_position": 0.04},
    )()
    runtime.target = np.array([0.1] * 7 + [0.04])
    controller = type(
        "Controller",
        (),
        {"ee_pos": np.zeros(3), "ee_ori_mat": np.eye(3)},
    )()
    robot = type("Robot", (), {"controller": controller})()
    runtime.environment = type(
        "Environment",
        (),
        {"env": type("Inner", (), {"robots": [robot]})()},
    )()
    runtime._target_eef_pose = lambda _joints: (np.array([0.025, -0.05, 0.1]), np.eye(3))
    runtime.orientation_error = lambda _target, _measured: np.array([0.25, -0.5, 1.0])

    action = runtime._action()

    assert np.allclose(action, [0.5, -1.0, 2.0, 0.5, -1.0, 2.0, 0.0])
