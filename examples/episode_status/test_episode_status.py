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

"""The supported sequence schema retains the nullable label wire contract."""

import sys

from dimos_cdrgen.registry import decode, encode, schema
from dimos_generated.dimos_msgs.msg import EpisodeStatus
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


@pytest.fixture(autouse=True)
def restore_reference_types_module():
    previous = sys.modules.get("rosbags.usertypes")
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop("rosbags.usertypes", None)
        else:
            sys.modules["rosbags.usertypes"] = previous


@pytest.mark.parametrize("labels", [[], [""], ["inspection"]])
def test_nullable_labels_and_signed_counters_preserve_bounded_wire_layout(labels):
    message = EpisodeStatus(
        ts=17.25,
        state="recording",
        episodes_saved=-2,
        episodes_discarded=-1,
        last_event="start",
        task_label=labels,
    )
    reference = get_typestore(Stores.EMPTY)
    bounded = schema(message.__msgtype__).replace("string[] task_label", "string[<=1] task_label")
    reference.register(get_types_from_msg(bounded, message.__msgtype__))
    wire = encode(message)
    assert wire == bytes(reference.serialize_cdr(message, message.__msgtype__))
    restored = decode(wire, EpisodeStatus)
    assert restored.task_label == labels
    assert (restored.ts, restored.state, restored.episodes_saved, restored.episodes_discarded) == (
        17.25,
        "recording",
        -2,
        -1,
    )
    assert restored.last_event == "start"
