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

from typing import Any

from dimos.core.global_config import GlobalConfig
from dimos.teleop.phone import phone_teleop_module
from dimos.teleop.phone.phone_teleop_module import PhoneTeleopConfig, PhoneTeleopModule


def test_phone_teleop_web_server_uses_module_global_config_listen_host(monkeypatch) -> None:
    captured: dict[str, Any] = {}

    class FakeRobotWebInterface:
        def __init__(self, **kwargs: Any) -> None:
            captured.update(kwargs)

    monkeypatch.setattr(phone_teleop_module, "RobotWebInterface", FakeRobotWebInterface)

    module = PhoneTeleopModule.__new__(PhoneTeleopModule)
    module.config = PhoneTeleopConfig(
        server_port=8444,
        g=GlobalConfig(listen_host="0.0.0.0"),
    )

    module._create_web_server()

    assert captured == {"host": "0.0.0.0", "port": 8444}
