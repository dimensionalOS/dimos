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

from typing import Any

from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray


def detection_array_to_rerun(
    detections: Detection2DArray,
    color: tuple[int, int, int, int],
    draw_order: float,
) -> Any:
    """Convert a Detection2DArray to a Rerun Boxes2D 2D overlay.

    Args:
        detections: The Detection2DArray message to visualize.
        color: RGBA color for all boxes.
        draw_order: Rerun draw order; higher values render on top.
    """
    import rerun as rr  # 延迟导入 Rerun，避免非 viewer 路径承担额外导入成本

    boxes: list[list[float]] = []
    labels: list[str] = []
    colors: list[list[int]] = []

    for index, detection in enumerate(detections.detections):
        bbox = detection.bbox
        center = bbox.center.position
        x = float(center.x - bbox.size_x / 2.0)
        y = float(center.y - bbox.size_y / 2.0)
        width = float(bbox.size_x)
        height = float(bbox.size_y)
        boxes.append([x, y, width, height])
        labels.append(str(detection.id or index))  # 优先显示 detection.id，没有 id 时显示当前序号
        colors.append(list(color))

    return rr.Boxes2D(  # 空 boxes 会清除 viewer 里的旧框
        array=boxes,
        array_format=rr.Box2DFormat.XYWH,
        colors=colors,
        labels=labels,
        show_labels=True,
        draw_order=draw_order,
    )


def detections_overlay(detections: Detection2DArray) -> Any:
    """Yellow semi-transparent overlay for YOLO candidate bboxes (draw_order=90)."""
    return detection_array_to_rerun(detections, (255, 190, 0, 180), 90.0)


def selected_bbox_overlay(detections: Detection2DArray) -> Any:
    """Solid green overlay for the RPC-selected bbox (draw_order=100)."""
    return detection_array_to_rerun(detections, (0, 255, 0, 255), 100.0)


def yoloe_overlay(detections: Detection2DArray) -> Any:
    """Cyan overlay for YOLOE tracking detections (draw_order=95)."""
    return detection_array_to_rerun(detections, (0, 220, 255, 220), 95.0)


__all__ = [
    "detection_array_to_rerun",
    "detections_overlay",
    "selected_bbox_overlay",
    "yoloe_overlay",
]
