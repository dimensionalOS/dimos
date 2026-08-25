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

import pytest_mock

from dimos.imitation.collection.recorder import CollectionRecorder, CollectionRecorderConfig


async def test_poseless_collection_stream_skips_pose_lookup(
    mocker: pytest_mock.MockerFixture,
) -> None:
    recorder = mocker.MagicMock(spec=CollectionRecorder)
    recorder.config = CollectionRecorderConfig(poseless_streams=["commands"])

    pose = await CollectionRecorder._resolve_pose(recorder, "commands", object(), 1.0)

    assert pose is None
