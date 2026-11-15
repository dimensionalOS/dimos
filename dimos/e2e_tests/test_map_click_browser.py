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

"""Browser-to-bridge coverage for map clicks and path-driven cancellation."""

from collections.abc import Iterator
import threading
from typing import NamedTuple

import cv2
import numpy as np
import pytest
from reactivex.disposable import Disposable

from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.web.cockpit import Map2D, Map3D, Row, cockpit
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.gen_costmap_fixtures import grid_msg
from dimos.web.relay_bridge.module_test_support import wait_until
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule

pytest.importorskip("playwright")

from playwright.sync_api import Locator, Page, expect, sync_playwright

pytestmark = pytest.mark.web_browser

CANVAS = '[data-testid="map2d-global_costmap-canvas"]'
# 2x2 cells at 0.5 m with the origin corner at (0.25, -0.5): world centre (0.75, 0).
GRID = grid_msg([[0, 50], [100, -1]], 0.5, 0.25, -0.5, 0.0)
# One elevated voxel: the camera fits it at the canvas centre, and its x/y
# differs from where that same ray hits the floor.
CLOUD = PointCloud2.from_numpy(np.array([[2.5, 3.5, 0.5]], np.float32), frame_id="world")


class MapBridge(NamedTuple):
    url: str
    module: RelayBridgeModule
    path: pLCMTransport


def _path(*xy: tuple[float, float]) -> Path:
    poses = [
        PoseStamped(ts=1.0, position=[x, y, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]) for x, y in xy
    ]
    return Path(ts=1.0, frame_id="world", poses=poses)


@pytest.fixture
def map_bridge() -> Iterator[MapBridge]:
    layout = Row(
        Map2D(pose=None, path="path", click="clicked_point", stop="stop_movement"),
        Map3D(pose=None, path="path", click="clicked_point", res=1.0),
    )
    (atom,) = cockpit(layout=layout).blueprints
    module = atom.module(local_port=0, open_browser=False, robot_id="map-click-e2e", **atom.kwargs)
    transports: list[pLCMTransport] = []
    publishers: dict[str, pLCMTransport] = {}
    for ch in ("global_costmap", "global_map", "path"):
        topic = f"/map_click_e2e/{ch}"
        bridge_side = pLCMTransport(topic)
        bridge_side.start()
        getattr(module, ch).transport = bridge_side
        publishers[ch] = pLCMTransport(topic)
        publishers[ch].start()
        transports += [bridge_side, publishers[ch]]
    stop = threading.Event()

    def mapper() -> None:
        # Frames flow only once the bridge lazily subscribes, so keep them coming.
        while not stop.is_set():
            publishers["global_costmap"].publish(GRID)
            publishers["global_map"].publish(CLOUD)
            stop.wait(0.5)

    thread = threading.Thread(target=mapper, daemon=True)
    thread.start()
    try:
        module.start()
        relay = module._relay
        assert relay is not None and relay.info is not None
        yield MapBridge(relay.info.open_url, module, publishers["path"])
    finally:
        stop.set()
        thread.join(timeout=2)
        stop_module(module)
        for transport in transports:
            transport.stop()


@pytest.fixture
def chromium_page() -> Iterator[Page]:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        try:
            yield browser.new_page()
        finally:
            browser.close()


def test_click_to_goal_and_cancel(map_bridge: MapBridge, chromium_page: Page) -> None:
    clicks: list[PointStamped] = []
    stops: list[bool] = []
    map_bridge.module.register_disposable(
        Disposable(map_bridge.module.clicked_point.subscribe(clicks.append))
    )
    map_bridge.module.register_disposable(
        Disposable(map_bridge.module.stop_movement.subscribe(lambda msg: stops.append(msg.data)))
    )

    chromium_page.goto(map_bridge.url)
    # The first grid sizes the backing store from the layout (300 is the
    # unsized default): the map is drawn once that happens.
    chromium_page.wait_for_function(
        f"""() => {{
          const canvas = document.querySelector('{CANVAS}');
          return canvas !== null && canvas.width !== 300;
        }}""",
        timeout=120_000,
    )
    chromium_page.click(CANVAS)
    assert wait_until(lambda: len(clicks) == 1, timeout=15.0)
    assert clicks[0].x == pytest.approx(0.75, abs=0.01)
    assert clicks[0].y == pytest.approx(0.0, abs=0.01)
    assert clicks[0].frame_id == "world"

    cancel = chromium_page.get_by_test_id("map2d-global_costmap-cancel")
    expect(cancel).to_have_count(0)
    map_bridge.path.publish(_path((0.3, -0.4), (0.75, 0.0), (1.2, 0.4)))
    expect(cancel).to_be_visible(timeout=30_000)
    cancel.click()
    assert wait_until(lambda: stops == [True], timeout=15.0)

    map_bridge.path.publish(Path())
    expect(cancel).to_have_count(0, timeout=30_000)


def test_voxel_and_ground_clicks_to_goals(map_bridge: MapBridge, chromium_page: Page) -> None:
    clicks: list[PointStamped] = []
    map_bridge.module.register_disposable(
        Disposable(map_bridge.module.clicked_point.subscribe(clicks.append))
    )
    chromium_page.goto(map_bridge.url)
    canvas = chromium_page.get_by_test_id("map3d-global_map-canvas")
    expect(canvas).to_have_attribute("data-voxels", "1", timeout=120_000)
    canvas.click()
    assert wait_until(lambda: len(clicks) == 1, timeout=15.0)
    assert (clicks[0].x, clicks[0].y, clicks[0].z) == pytest.approx((2.5, 3.5, 0.0))
    assert clicks[0].frame_id == "world"

    # A corner of the canvas misses the centred sprite and points downward.
    box = canvas.bounding_box()
    assert box is not None
    canvas.click(position={"x": 10, "y": box["height"] - 10})
    assert wait_until(lambda: len(clicks) == 2, timeout=15.0)
    assert (clicks[1].x, clicks[1].y) != pytest.approx((2.5, 3.5))
    assert np.isfinite([clicks[1].x, clicks[1].y]).all()
    assert clicks[1].z == 0.0
    assert clicks[1].frame_id == "world"
    expect(chromium_page.locator('[role="alert"]')).to_have_count(0)


def _path_pixels(canvas: Locator) -> int:
    image = cv2.imdecode(np.frombuffer(canvas.screenshot(), np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    # Match the path's green hue across rendering/colour-management differences.
    # Saturation and brightness exclude the dark grid; the voxel has a different hue.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return int(cv2.countNonZero(cv2.inRange(hsv, (50, 80, 80), (80, 255, 255))))


def test_3d_path_draws_and_clears(map_bridge: MapBridge, chromium_page: Page) -> None:
    chromium_page.goto(map_bridge.url)
    canvas = chromium_page.get_by_test_id("map3d-global_map-canvas")
    expect(canvas).to_have_attribute("data-voxels", "1", timeout=120_000)
    assert _path_pixels(canvas) == 0

    map_bridge.path.publish(_path((1.5, 3.0), (2.5, 3.5), (3.5, 3.0)))
    assert wait_until(lambda: _path_pixels(canvas) > 20, timeout=15.0)
    map_bridge.path.publish(Path())
    assert wait_until(lambda: _path_pixels(canvas) == 0, timeout=15.0)
    expect(chromium_page.locator('[role="alert"]')).to_have_count(0)
