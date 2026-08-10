# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.detection.type.detection3d.pointcloud_filters import PointCloudFilter

# Legend prepended once per encoding pass: how to read the per-frame detection
# lines, plus closed-form decision procedures for the spatial reasoning an
# agent is asked to do over a semantic object map. Procedures are deliberately
# compare-and-sign recipes (no free-form trigonometry) so a model can follow
# them step by step.
_AGENT_LEGEND = """\
SEMANTIC OBJECT MAP -- reading guide and reasoning procedures.
Each observation below is one camera frame; each entry reads
"name#k conf=C pos=(x, y, z)": one detection of class name at world-frame
position (x, y, z) in meters (+x east, +y north, z up). The same physical
object is re-detected on many frames, so it appears as many entries with
slightly different positions; #k is its instance id — entries sharing a
tag (class AND id) are sightings of the SAME physical object (ids come
from an exact greedy scan: a detection joins the first same-class instance
whose running-mean position lies within 1.5 m, else it founds the next id).

Use these procedures exactly; do every computation numerically, step by step.

P1 BUILD THE OBJECT LIST FIRST (before answering anything):
  a. Group entries by their full tag (class AND id). Each tag is ONE
     physical object: position = the mean (x, y) of its entries, sighting
     count = the number of entries carrying the tag.
  b. Different ids of the same class are DISTINCT objects even when their
     means sit only one-to-two meters apart. Never merge tags, and never
     measure a distance between entries of the same tag.
  c. Then DELETE noise, before anything else uses the table: tags with
     fewer than 3 entries, classes implausible indoors (vehicles, street
     objects, wild animals — a plush toy is a household object, not an
     animal — and outdoor sports gear), and inherently mobile classes
     (people). A deleted tag must never appear in any later step or answer
     — if a class keeps only one tag, that survivor is THE object of its
     class.
  d. Write out the final table (tag, mean x, mean y, count) and answer
     every question from that table only.
P2 DISTANCES are horizontal: dist = sqrt((x1-x2)^2 + (y1-y2)^2) between
  cluster positions (robot-relative distances use the robot pose of P4).
  Answer numeric questions with the computed number only; never refuse.
P3 COMPASS direction of A from B: dx = xA-xB, dy = yA-yB. Write down both
  products 2.41*|dx| and 2.41*|dy| before comparing anything. A bigger |dx|
  alone does NOT mean east/west: the pure directions need a 2.41x dominance,
  otherwise the answer is diagonal.
  If |dx| > 2.41*|dy| -> "east" if dx>0 else "west".
  Else if |dy| > 2.41*|dx| -> "north" if dy>0 else "south".
  Else diagonal -> "northeast" (dx>0, dy>0), "northwest" (dx<0, dy>0),
  "southeast" (dx>0, dy<0), "southwest" (dx<0, dy<0).
P4 ROBOT POSE: the LAST odom observation shown. Its pos gives your (xr, yr);
  its euler [roll, pitch, yaw] is in DEGREES -- your heading is th = yaw
  (use degree-mode trig, or convert to radians first).
P5 EGO FRAME (is an object ahead/behind/left/right of YOU): never judge from
  raw world coordinates; transform first. With dx = xo-xr, dy = yo-yr to the
  object cluster: fwd = dx*cos(th) + dy*sin(th); left = -dx*sin(th) +
  dy*cos(th). Write both numbers down. Then:
  if |fwd| >= |left| -> "ahead" if fwd > 0 else "behind";
  otherwise -> "left" if left > 0 else "right".
P6 COUNTS AND ZONES: count DISTINCT clusters (after P1), not entries. For a
  zone question, test each cluster's mean (x, y) against the zone's
  inequalities and count/list only the clusters that satisfy them.
P7 NEAREST / NEXT-TO: write a full table — one line per candidate class with
  its distance from the reference point (to the class's nearest cluster) —
  then answer the class with the smallest distance. Never answer before the
  table is complete. Recency and salience are irrelevant: a prominent or
  recently-seen object is often NOT the nearest — before answering, verify
  no other class in the table has a smaller distance.
P8 LISTS (which classes exist / lie within a radius / lie in a zone): apply
  P1 first, then list each qualifying class exactly once, comma-separated.
P9 BETWEEN A and B: first admit only candidates strictly between the two
  anchors: a cluster qualifies only if BOTH its distance to A and its
  distance to B are smaller than the A-to-B distance (deleted noise tags
  never qualify; a cluster on the far side of an anchor fails this test
  even if it lies on the A-B line). Among qualifying clusters, answer the
  class of the one nearest the midpoint ((xA+xB)/2, (yA+yB)/2).
"""


class ImageDetections3DPC(ImageDetections[Detection3DPC]):
    """Specialized class for 3D detections in an image."""

    # Max frame timestamp seen by agent_encode; a non-increasing ts means a
    # new encoding pass started (frames are re-encoded from the top).
    _agent_last_ts: float | None = None
    # Per-pass online instance clustering: class name -> per-instance
    # [sum x, sum y, count], in founding order. A detection joins the first
    # instance of its class whose running mean lies within 1.5 m, else it
    # founds a new one. Reset at each new encoding pass.
    _agent_instances: ClassVar[dict[str, list[list[float]]]] = {}

    def agent_encode(self) -> list[dict[str, str]]:
        """Model-legible encoding: compact per-frame detection lines with
        greedily-assigned instance-id tags (``name#k``).

        Prepends the reasoning-procedure legend once per encoding pass.
        Frame timestamps are monotonic within a pass, so a non-increasing
        timestamp signals a fresh pass (ponytail: monotonic-ts heuristic;
        move legend + clustering state into the encoder loop if streams
        ever interleave).
        """
        cls = ImageDetections3DPC
        ts = self.image.ts
        new_pass = cls._agent_last_ts is None or ts <= cls._agent_last_ts
        cls._agent_last_ts = ts
        if new_pass:
            cls._agent_instances = {}

        parts: list[str] = []
        for det in self.detections:
            x, y = det.center.x, det.center.y
            instances = cls._agent_instances.setdefault(det.name, [])
            for k, inst in enumerate(instances):  # noqa: B007 - k used after break
                if math.hypot(x - inst[0] / inst[2], y - inst[1] / inst[2]) <= 1.5:
                    inst[0] += x
                    inst[1] += y
                    inst[2] += 1
                    break
            else:
                instances.append([x, y, 1.0])
                k = len(instances) - 1
            parts.append(
                f"{det.name}#{k + 1} conf={det.confidence:.2f} "
                f"pos=({x:.2f}, {y:.2f}, {det.center.z:.2f})"
            )
        blocks = [{"type": "text", "text": "; ".join(parts) or "no detections"}]
        if new_pass:
            blocks.insert(0, {"type": "text", "text": _AGENT_LEGEND})
        return blocks

    @classmethod
    def from_2d(
        cls,
        detections_2d: ImageDetections2D,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> ImageDetections3DPC:
        """Project every 2D detection into 3D, dropping any that yield no valid points."""
        detections_3d = [
            d3d
            for det in detections_2d
            if (
                d3d := Detection3DPC.from_2d(
                    det,
                    world_pointcloud,
                    camera_info,
                    world_to_optical_transform,
                    filters,
                )
            )
            is not None
        ]
        return cls(image=detections_2d.image, detections=detections_3d)

    @classmethod
    def from_depth(
        cls,
        detections_2d: ImageDetections2D,
        depth: Image,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> ImageDetections3DPC:
        """Unproject every detection's depth pixels into 3D, dropping empty results."""
        detections_3d = [
            d3d
            for det in detections_2d
            if (
                d3d := Detection3DPC.from_depth(
                    det,
                    depth,
                    camera_info,
                    world_to_optical_transform,
                    filters,
                )
            )
            is not None
        ]
        return cls(image=detections_2d.image, detections=detections_3d)
