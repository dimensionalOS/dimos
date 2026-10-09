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

from __future__ import annotations

from dimos.msgs.sim_msgs.Contacts import Contact, Contacts


def test_round_trip_keeps_contacts_and_stamp() -> None:
    msg = Contacts([Contact("foot", "floor"), Contact("trunk", "wall")], ts=12.5)
    back = Contacts.lcm_decode(msg.lcm_encode())
    assert back.contacts == msg.contacts
    assert back.ts == 12.5


def test_empty_set_round_trips() -> None:
    back = Contacts.lcm_decode(Contacts(ts=1.0).lcm_encode())
    assert back.contacts == []
    assert back.ts == 1.0
