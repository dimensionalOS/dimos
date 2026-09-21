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

"""Browser-to-bridge coverage for the 3D map panel: a voxel frame crosses
the wire, inflates in the page and draws through WebGL with pixels to show
for it.

Marked web_browser: excluded from the default suite (needs the
`browser-tests` dependency group, Playwright browsers, and built web dists).
Locally:
`uv run --group browser-tests pytest -m web_browser dimos/e2e_tests/test_map3d_browser.py`.
"""

from collections.abc import Callable

import cv2
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.web.cockpit import Map3D, cockpit

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect

pytestmark = pytest.mark.web_browser

CANVAS = '[data-testid="map3d-global_map-canvas"]'
# Three 16 m voxels a kilometre apart: fitting them backs the camera off
# ~2.6 km, so the frame only shows if the clip planes follow the fit, and the
# voxels are still ~4 px wide from there. Their heights put one voxel at each
# end of the height ramp and one in the middle.
CLOUD = PointCloud2.from_numpy(
    np.array([[-1000.0, 0.0, 0.0], [0.0, 0.0, 16.0], [1000.0, 0.0, 32.0]], np.float32),
    frame_id="world",
    timestamp=1.0,
)


@pytest.fixture(scope="module")
def map3d_page_url(serve_channel: Callable[..., str]) -> str:
    return serve_channel(
        cockpit(layout=Map3D(pose=None, res=16.0)),
        stream="global_map",
        topic="/map3d_e2e/global_map",
        message=CLOUD,
        robot_id="map3d-e2e",
    )


def test_voxel_map_renders(map3d_page_url: str, chromium_page: Page) -> None:
    chromium_page.goto(map3d_page_url)
    # The sink stamps the voxel count on the canvas after the scene drew it,
    # so this waits for the lazy three.js chunk, the inflate and WebGL.
    chromium_page.wait_for_selector(f'{CANVAS}[data-voxels="3"]', timeout=120_000)
    badge = chromium_page.get_by_test_id("map3d-global_map-badge")
    expect(badge).to_have_attribute("data-state", "live", timeout=30_000)
    expect(chromium_page.locator('[role="alert"]')).to_have_count(0)
    # Drawn means visible: the ramp paints the middle voxel bright green,
    # while the grid lines stay under 0x30 on the black background. A frame whose
    # geometry is clipped decodes and stamps just the same, but shows nothing.
    # Without a pose channel no follow button sits on the canvas.
    png = chromium_page.locator(CANVAS).screenshot()
    image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    bright = int((image.max(axis=2) > 128).sum())
    assert bright > 0, "no bright pixel on the canvas: the voxels were clipped or not drawn"
