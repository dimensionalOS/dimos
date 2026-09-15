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

from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.teleop.webxr.collection_prompts import CollectionPrompts


def test_confirmed_transitions_ignore_duplicates_and_idle_commands():
    prompts = CollectionPrompts()
    events = [
        ("init", "idle", 0, 0, None),
        ("save", "idle", 0, 0, None),
        ("discard", "idle", 0, 0, None),
        ("start", "recording", 0, 0, "Recording started"),
        ("save", "idle", 1, 0, "Episode saved"),
        ("start", "recording", 1, 0, "Recording started"),
        ("discard", "idle", 1, 1, "Recording canceled"),
        ("start", "recording", 1, 1, "Recording started"),
        ("start", "recording", 2, 1, "Recording started"),
    ]
    for ts, (event, state, saved, discarded, expected) in enumerate(events):
        status = EpisodeStatus(
            ts=ts, last_event=event, state=state, episodes_saved=saved, episodes_discarded=discarded
        )
        assert prompts.update(status) == expected
        assert prompts.update(status) is None
