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

"""The python config and the rust struct agree field for field."""

from pathlib import Path

from dimos.robot.deeprobotics.m20.controller.module import M20ControllerConfig

_RUST = Path(__file__).parent / "rust" / "src" / "module.rs"


def test_the_wire_config_has_every_rust_field() -> None:
    src = _RUST.read_text()
    struct = src.split("pub struct Config {", 1)[1].split("\n}", 1)[0]
    declared = {
        line.split(":")[0].strip().removeprefix("pub ")
        for line in struct.splitlines()
        if ": " in line and not line.strip().startswith(("//", "#"))
    }
    assert set(M20ControllerConfig(mode="owner").to_config_dict()) == declared


def test_what_the_native_process_publishes_reaches_the_python_side(mocker) -> None:
    """The native publishes on its outputs' zenoh keys from another session; Out.subscribe() alone never sees that."""
    import json

    from dimos.core.native_module import NativeModule
    from dimos.core.transport import ZenohTransport
    from dimos.msgs.std_msgs.String import String
    from dimos.robot.deeprobotics.m20.controller.module import M20Controller
    from dimos.utils.testing.waiting import wait_until

    mocker.patch.object(NativeModule, "start", lambda self: None)
    keys = {
        name: f"test/m20_controller_watch/{name}"
        for name in ("controller_state", "controller_report")
    }
    with M20Controller(mode="owner") as module:
        for name, key in keys.items():
            module.set_transport(name, ZenohTransport(key, String))
        module.start()
        status = json.dumps({"state": "RlControl", "gait": "0x1001"})
        native_state = ZenohTransport(keys["controller_state"], String)
        native_status = ZenohTransport(keys["controller_report"], String)

        def arrived() -> bool:
            native_state.publish(String("RlControl"))
            native_status.publish(String(status))
            return (
                module.controller_status() == "RlControl"
                and module.status().get("gait") == "0x1001"
            )

        wait_until(arrived, timeout=5.0, interval=0.05)
        native_state.stop()
        native_status.stop()
