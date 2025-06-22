# Copyright 2025 Dimensional Inc.
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

import time

import pytest

from dimos.multiprocess.actors2.recognition import Recognition
from dimos.multiprocess.actors2.video import Video


def test_video_introspection():
    print("\n" + Video.io())


@pytest.mark.asyncio
async def test_play_local():
    video = Video(video_name="office.mp4")
    recognition = Recognition(video.video_stream)
    video.play(frames=10)


@pytest.mark.asyncio
async def test_play_lcm():
    video = Video(video_name="office.mp4")
    videoframes = topic("/video/frames")
    video.video_stream.subscribe(lambda frame: videoframes.on_next(frame.get("frame_number")))


@pytest.mark.asyncio
async def test_play_dask():
    video = run_remote(Video, video_name="office.mp4")
    video.video_stream.subscribe(print)
