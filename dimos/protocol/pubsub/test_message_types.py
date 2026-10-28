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

from dimos.messages.geometry_msgs.msg import Vector3
from dimos.protocol.pubsub.message_types import resolve_message_type


def test_resolves_registered_canonical_and_short_names():
    assert resolve_message_type("geometry_msgs/msg/Vector3") is Vector3
    assert resolve_message_type("geometry_msgs.Vector3") is Vector3


def test_unknown_names_do_not_import_arbitrary_modules():
    assert resolve_message_type("unregistered.Vector3") is None
    assert resolve_message_type("dimos.msgs.geometry_msgs.Vector3") is None
