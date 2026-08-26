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

import pytest

from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus


@pytest.mark.parametrize("task_label", ["pick", None])
def test_episode_status_lcm_roundtrip_preserves_status_update(
    task_label: str | None,
) -> None:
    expected = EpisodeStatus(
        ts=12.25,
        state="recording",
        episodes_saved=2,
        episodes_discarded=1,
        last_event="start",
        task_label=task_label,
    )

    actual = EpisodeStatus.lcm_decode(expected.lcm_encode())

    assert actual == expected
