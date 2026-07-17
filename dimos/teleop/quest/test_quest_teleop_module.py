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

from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule


def test_quest_web_server_binds_all_interfaces_without_starting_server() -> None:
    module = QuestTeleopModule(server_port=9443)

    try:
        assert module._web_server.host == "0.0.0.0"
        assert module._web_server.port == 9443
    finally:
        module.stop()
