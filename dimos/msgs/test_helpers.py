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

from dimos_generated.geometry_msgs.msg import Pose

from dimos.msgs.helpers import resolve_msg_type


def test_installed_generated_type_resolves_by_schema_name() -> None:
    assert resolve_msg_type("geometry_msgs/msg/Pose") is Pose


def test_unknown_names_do_not_import_a_legacy_message_package() -> None:
    assert resolve_msg_type("geometry_msgs/msg/Nope") is None
    assert resolve_msg_type("geometry_msgs.Pose") is None
    assert resolve_msg_type("Bare") is None
