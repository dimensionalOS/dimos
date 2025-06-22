#!/usr/bin/env python3

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

from dimos.multiprocess.actors2.video import Video


def test_video_introspection():
    print("\n" + Video.io())


def test_video():
    """Test the Video module."""
    video = Video(video_name="office.mp4")

    # Test play method
    video.play(frames=10)

    # Test get_video_properties method
    properties = video.get_video_properties()

    assert properties["width"] > 0, "Width should be greater than 0"
    assert properties["height"] > 0, "Height should be greater than 0"
    assert properties["total_frames"] > 0, "Total frames should be greater than 0"

    print("Video properties:", properties)
