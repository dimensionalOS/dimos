from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
from unittest.mock import MagicMock


def _load_bridge_module():
    module_name = "test_nav_cmd_rclpy_bridge_module"
    module_path = Path(
        "/Users/afik_cohen/gt/dimos/crew/ace/dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/nav_cmd_rclpy_bridge.py"
    )

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda _node: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {"__init__": lambda self, *_args, **_kwargs: None})

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, **_kwargs: None})
    rclpy_qos.QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE=object())
    rclpy_qos.QoSDurabilityPolicy = types.SimpleNamespace(VOLATILE=object())

    drdds = types.ModuleType("drdds")
    drdds_msg = types.ModuleType("drdds.msg")
    drdds_msg.NavCmd = type("NavCmd", (), {})

    original_modules = {
        name: sys.modules.get(name)
        for name in ["rclpy", "rclpy.node", "rclpy.qos", "drdds", "drdds.msg", module_name]
    }
    original_argv = sys.argv[:]

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos
    sys.modules["drdds"] = drdds
    sys.modules["drdds.msg"] = drdds_msg
    sys.argv = ["nav_cmd_rclpy_bridge.py"]

    try:
        spec = importlib.util.spec_from_file_location(module_name, module_path)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.argv = original_argv
        for name, original in original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


def test_recv_packet_returns_none_on_clean_disconnect() -> None:
    module = _load_bridge_module()
    client = MagicMock()
    client.recv.return_value = b""

    assert module._recv_packet(client) is None
