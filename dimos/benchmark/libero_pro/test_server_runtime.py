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
        "robot0_eye_in_hand_image": np.zeros((128, 128, 3), dtype=np.uint8),
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
        runtime.frequency = 1_000
        runtime.horizon = 1
        runtime.target = np.zeros(8)
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
