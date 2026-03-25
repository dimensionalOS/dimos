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

"""Visualizer tests: query the go2_bigoffice replay DB and return images + poses."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import Batch, QualityWindow
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.florence import Florence2Model
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import get_data_dir

if TYPE_CHECKING:
    from collections.abc import Iterator

DB_PATH = get_data_dir() / "go2_bigoffice.db"


@pytest.fixture(scope="module")
def store() -> Iterator[SqliteStore]:
    db = SqliteStore(path=str(DB_PATH))
    with db:
        yield db


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    return CLIPModel()


# GENERAL ON VIS
#
# these need to be functions that are easily called to render latest query results in different ways
# I can imagine querying for multiple things and I want previous visualization to dissaper, being replaced
# by latest results. we might do this transparently so agent queries are also visible in real time
#
# Actual prerequisite for this is a good python API
#
# I don't actually know how vis for this should look like, is visualizer just a consumer of a stream?
#
# visualize(embedded.search(clip.embed_text("bottle"), k=10))
#
# this means it could work with live (realtime) queries as well (memory supports live (ongoing) queries)
# visualize(detections.search("bottle").live())
#


@pytest.mark.tool
class TestVisualizer:
    def test_db(self, store: SqliteStore) -> None:
        print("Available streams:", store.streams)

    def test_first_image_with_pose(self, store: SqliteStore) -> None:
        video = store.stream("color_image", Image)
        obs = video.first()

        assert isinstance(obs.data, Image)
        assert obs.pose is not None
        print(f"ts={obs.ts}, pose={obs.pose}, image={obs.data}")

    # we search for 10 images matching "a door"
    #
    # VIS GOAL: draw each image in 3d space in the position of capture
    # potentially also draw them in a grid with similarity scores, or something like that
    def test_search_by_text(self, store: SqliteStore, clip: CLIPModel) -> None:
        """Search embedded frames with a text query."""
        embedded = store.streams.color_image_embedded
        for obs in embedded.search(clip.embed_text("a door"), k=10):
            # embedded observation here
            print(obs.similarity)  # similarity score
            print(obs.data)  # image
            print(obs.pose)  # pose

    # we search for all images near some global location
    #
    # VIS GOAL: many images, draw just poses for each
    def test_search_near_pose(self, store: SqliteStore) -> None:
        """Find images near a pose within a time window."""
        video = store.streams.color_image
        lidar = store.streams.lidar
        # find images in a 5m radius near the first frame's pose
        for obs in video.near(video.first().pose, radius=5.0):
            print(f"ts={obs.ts:.2f} pose={obs.pose}")
            print(lidar.at(obs.ts).first().data)  # get a related lidar frame (can try and draw)

    # we semantically search, then detect with a detection model
    #
    # VIS GOAL: draw 2d detections somehow, or project into 3d, draw 3d bounding boxes
    def test_detect_objects(self, store: SqliteStore, clip: CLIPModel) -> None:
        """CLIP pre-filter + VLM detection on top candidates."""
        from dimos.models.vl.moondream import MoondreamVlModel

        vlm = MoondreamVlModel()
        embedded = store.streams.color_image_embedded
        lidar = store.streams.lidar

        for obs in embedded.search(clip.embed_text("bottle"), k=10).map(
            lambda obs: obs.derive(data=vlm.query_detections(obs.data, "bottle"))
        ):
            print(f"ts={obs.ts:.2f} sim={obs.similarity:.3f} pose={obs.pose}")
            for det in obs.data.detections:
                print(det)
                print(
                    lidar.at(obs.ts).first().data
                )  # get a related lidar frame (can try and project)

    # draw the path robot took
    #
    # VIS GOAL: I should be able to draw these poses individually or as a path
    def test_search_reconstruct_full_path(self, store: SqliteStore) -> None:
        for obs in store.streams.color_image_embedded:
            assert obs.pose is not None

    # we can also generate textxual descriptions of images returned from queries
    # or in real time as robot runs
    #
    # VIS GOAL: how dow e want to draw those?
    def test_agent_visual_description_passive(self, store: SqliteStore) -> None:
        florence = Florence2Model()
        with florence:
            pipeline = store.streams.color_image.transform(
                QualityWindow(lambda img: img.sharpness, window=5.0)
                # we are batch processing images here,
                # so we can use the more efficient batch captioning API
                # (instead of using .map() and calling caption() for each image,
            ).transform(Batch(lambda imgs: florence.caption_batch(*imgs)))
            # this can be stored, further embedded etc

            for obs in pipeline:
                print(obs.ts, obs.data)
