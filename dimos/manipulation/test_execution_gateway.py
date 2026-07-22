"""Black-box tests for coordinator gateway normalization."""

from typing import Any

from dimos.manipulation.execution_gateway import ControlCoordinatorGateway
from dimos.manipulation.execution_models import Outcome
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


class Client:
    def __init__(self, value: Any) -> None:
        self.value = value
        self.calls: list[tuple[str, str, dict[str, Any]]] = []

    def task_invoke(self, task: str, method: str, kwargs: dict[str, Any]) -> Any:
        self.calls.append((task, method, kwargs))
        return self.value

    def set_gripper_position(self, hardware_id: str, position: float) -> Any:
        self.calls.append((hardware_id, "set_gripper_position", {"position": position}))
        return self.value

    def get_gripper_position(self, hardware_id: str) -> Any:
        self.calls.append((hardware_id, "get_gripper_position", {}))
        return self.value


def test_gateway_normalizes_execute_cancel_and_status() -> None:
    client = Client(True)
    gateway = ControlCoordinatorGateway(client)
    assert gateway.execute("task", {"trajectory": "request"}) == Outcome.ACCEPTED
    assert gateway.cancel("task") == Outcome.CANCELLED

    client.value = TrajectoryState.EXECUTING
    assert gateway.status("task") == Outcome.RUNNING
    assert client.calls[0] == ("task", "execute", {"trajectory": "request"})

    client.value = False
    assert gateway.cancel("task") == Outcome.INACTIVE


def test_gateway_normalizes_unknown_values_safely() -> None:
    client = Client("not-a-state")
    gateway = ControlCoordinatorGateway(client)
    assert gateway.status("task") == Outcome.UNKNOWN
    client.value = object()
    assert gateway.execute("task", {}) == Outcome.UNKNOWN


def test_gateway_uses_top_level_gripper_rpc_methods() -> None:
    client = Client(True)
    gateway = ControlCoordinatorGateway(client)
    assert gateway.set_gripper_position("gripper", 0.4) == Outcome.ACCEPTED
    client.value = 0.6
    assert gateway.get_gripper_position("gripper") == 0.6
    assert client.calls == [
        ("gripper", "set_gripper_position", {"position": 0.4}),
        ("gripper", "get_gripper_position", {}),
    ]
