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

"""Generated CDR over LCM browser E2E.

A typed channel advertises its complete ROS2 schema. The browser decodes the
Python CDR payload from the manifest alone; LCM remains the raw transport.
Requires the browser-tests group, installed browsers and built web dists.
"""

from collections.abc import Callable
from pathlib import Path

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.msgs.time import time_from_seconds
from dimos.web.cockpit import Channel, cockpit

pytest.importorskip("playwright")

from playwright.sync_api import Page, expect

pytestmark = pytest.mark.web_browser

TOPIC = "/lcm_channel_e2e/lcm_pose"

POSE = PoseStamped(
    header=Header(stamp=time_from_seconds(42.5), frame_id="map"),
    pose=Pose(position=Point(x=1.5, y=-2.5, z=0.25), orientation=Quaternion(w=1)),
)

# A page whose SDK and relay live on another origin, reading the decoded CDR
# value's fields straight off the store snapshot.
SDK_PAGE = """<!DOCTYPE html>
<html>
  <body>
    <pre id="pose">(no data)</pre>
    <script type="module">
      import { connect } from "%(url)ssdk.js";
      const session = connect({ url: "%(url)s" });
      session.subscribe("lcm_pose", (snapshot) => {
        const v = snapshot.slot?.value;
        if (v === undefined || v === null) return;
        document.querySelector("#pose").textContent =
          `x=${v.pose.position.x} frame=${v.header.frame_id} nanosec=${v.header.stamp.nanosec}`;
      });
    </script>
  </body>
</html>
"""


@pytest.fixture(scope="module")
def cockpit_url(serve_channel: Callable[..., str]) -> str:
    blueprint = cockpit(channels=[Channel("lcm_pose", PoseStamped, max_hz=20.0)])
    return serve_channel(
        blueprint, stream="lcm_pose", topic=TOPIC, message=POSE, robot_id="lcm-e2e"
    )


def test_cockpit_channel_table_decodes_the_lcm_channel(
    cockpit_url: str, chromium_page: Page
) -> None:
    chromium_page.goto(cockpit_url)
    chromium_page.get_by_test_id("view-channels").click()
    expect(chromium_page.get_by_test_id("ch-lcm_pose-seq")).not_to_have_text("-", timeout=120_000)
    value = chromium_page.get_by_test_id("ch-lcm_pose-value")
    # The preview is the schema-driven decoder's: generated ROS2 fields, decoded from the advertised schema.
    expect(value).to_contain_text("position: {x: 1.5, y: -2.5, z: 0.25}")
    expect(value).to_contain_text('frame_id: "map"')
    expect(chromium_page.get_by_test_id("ch-lcm_pose-decode-error")).to_have_count(0)
    expect(chromium_page.get_by_text("geometry_msgs/msg/PoseStamped.cdr.v1")).to_be_visible()


def test_sdk_page_reads_the_decoded_lcm_fields(
    cockpit_url: str, chromium_page: Page, tmp_path: Path
) -> None:
    page = tmp_path / "index.html"
    page.write_text(SDK_PAGE % {"url": cockpit_url})
    chromium_page.goto(page.as_uri())
    expect(chromium_page.locator("#pose")).to_have_text(
        "x=1.5 frame=map nanosec=500000000", timeout=120_000
    )
