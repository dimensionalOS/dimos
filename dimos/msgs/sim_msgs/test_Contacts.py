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

from dimos_generated.dimos_msgs.msg import Contact, Contacts
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode, encode
import pytest

from dimos.msgs.time import time_from_seconds, to_seconds


@pytest.mark.parametrize(
    "contacts", [[], [Contact(part="foot", kind="floor"), Contact(part="trunk", kind="wall")]]
)
def test_cdr_retains_semantic_contacts_and_source_stamp(contacts):
    value = Contacts(header=Header(stamp=time_from_seconds(12.5), frame_id=""), contacts=contacts)
    decoded = decode(encode(value), Contacts)
    assert decoded.contacts == contacts
    assert to_seconds(decoded.header.stamp) == 12.5
