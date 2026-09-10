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

"""Rerun visualization helpers for Spot blueprints."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from collections.abc import Callable

    import rerun.blueprint as rrb

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.visualization.rerun.bridge import RerunData

# Optical tf frame_id (SpotHighLevelConfig defaults) -> stream-name suffix.
_OPTICAL_FRAME_TO_SUFFIX = {
    "frontleft_camera_optical": "front_left",
    "frontright_camera_optical": "front_right",
    "left_camera_optical": "left",
    "right_camera_optical": "right",
    "back_camera_optical": "back",
}

# The five cameras mount ~0.3 m apart on Spot's body; a full 1.0 m image plane
# makes neighbouring frustums overlap in the 3D view, so draw them shorter.
_FRUSTUM_PLANE_DISTANCE = 0.3

# Spot's body is ~1.1 m long, 0.5 m wide, 0.19 m tall and base_link sits at its
# center, so a box that size on the frame stands in for the robot.
_SPOT_BODY_SIZE = (1.1, 0.5, 0.19)
_SPOT_GREEN = (0, 255, 0)

# Mirrors the tf stream so a camera pose can be looked up at the moment its
# image was captured. Module-level, not an attribute of the override objects:
# those get pickled out to the bridge's worker and would each carry a copy.
_tf_buffer = MultiTBuffer()


def _anchor_frame() -> str | None:
    """The frame that is a parent but never a child - the one that never moves.

    Everything else in the tf tree hangs off it, so a pose expressed against it
    stays put no matter what the robot does afterwards. Discovered rather than
    named because the odometry frame is configurable. None while the tree is
    still filling in, or if it has more than one root.
    """
    parents = {parent for parent, _ in _tf_buffer.buffers}
    children = {child for _, child in _tf_buffer.buffers}
    roots = parents - children
    return roots.pop() if len(roots) == 1 else None


def _grayscale_origin(suffix: str) -> str:
    return f"world/grayscale_image_{suffix}"


def _depth_origin(suffix: str) -> str:
    return f"world/depth_image_{suffix}"


def _camera_entity(origin: str) -> str:
    """Child entity holding the projection and the pixels, under the camera's pose.

    A Pinhole is a parent link, and rerun gives an entity exactly one — so an
    entity carrying a projection cannot also carry the camera's pose. One level
    down the pixels inherit the pose logged above them.
    """
    return f"{origin}/camera"


def _tf_to_rerun(tf_message: TFMessage) -> RerunData:
    """Log tf as usual, and keep a copy for the camera pose lookups."""
    _tf_buffer.receive_tfmessage(tf_message)
    return tf_message.to_rerun()


def _camera_info_pinhole(camera_info: CameraInfo, origin: Callable[[str], str]) -> RerunData | None:
    """Re-emit a shared CameraInfo onto its camera's image entity as a Pinhole."""
    suffix = _OPTICAL_FRAME_TO_SUFFIX.get(camera_info.frame_id)
    if suffix is None:
        return None
    return camera_info.to_rerun(
        image_plane_distance=_FRUSTUM_PLANE_DISTANCE,
        image_topic=_camera_entity(origin(suffix)),
    )


# Module-level (not closures) so the RerunBridgeModule config stays picklable
# when it is shipped to its worker process.
def _grayscale_info_to_pinhole(camera_info: CameraInfo) -> RerunData | None:
    return _camera_info_pinhole(camera_info, _grayscale_origin)


def _depth_info_to_pinhole(camera_info: CameraInfo) -> RerunData | None:
    return _camera_info_pinhole(camera_info, _depth_origin)


def spot_camera_visual_overrides() -> dict[str, Callable[[Any], RerunData | None]]:
    """Give each camera its own frustum, posed where it was at capture time.

    ``grayscale_info``/``depth_info`` are shared across all five cameras, so
    their default Pinhole lands on one throwaway entity and no camera renders a
    frustum. Route each message onto its camera's entity (picked by the
    CameraInfo's optical ``frame_id``) so every image plane gets a projection,
    and hand the images a pose of their own — see :class:`_ImageBakedIntoAnchor`.
    """
    overrides: dict[str, Callable[[Any], RerunData | None]] = {
        "world/tf": _tf_to_rerun,
        "world/grayscale_info": _grayscale_info_to_pinhole,
        "world/depth_info": _depth_info_to_pinhole,
    }
    for suffix in _OPTICAL_FRAME_TO_SUFFIX.values():
        for origin in (_grayscale_origin, _depth_origin):
            overrides[origin(suffix)] = _ImageBakedIntoAnchor(origin(suffix))
    return overrides


def _spot_body_box(rerun_module: Any) -> list[Any]:
    """A green box at base_link standing in for Spot's body."""
    return [
        rerun_module.Transform3D(parent_frame="tf#/base_link"),
        rerun_module.Boxes3D(
            centers=[(0.0, 0.0, 0.0)],
            sizes=[_SPOT_BODY_SIZE],
            colors=[_SPOT_GREEN],
            fill_mode="solid",
        ),
    ]


def spot_body_static_overrides() -> dict[str, Callable[[Any], Any]]:
    """Draw a green body box anchored to the moving base_link frame."""
    return {"world/spot_body": _spot_body_box}


def _camera_view(origin: str, name: str) -> rrb.Spatial2DView:
    import rerun.blueprint as rrb

    return rrb.Spatial2DView(origin=_camera_entity(origin), name=name)


def spot_camera_layout() -> rrb.Blueprint:
    """A 3D view of Spot beside tabbed 2D camera panels.

    Left: the robot's tf tree and every camera frustum in place. Right: tabs of
    grayscale/depth feeds labelled by URDF mount position.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    world_view = rrb.Spatial3DView(
        origin="world",
        name="3D",
        background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
        line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
    )

    # Each rrb view may live in exactly one place in the blueprint tree, so every
    # container below builds its own fresh view instances rather than sharing one.
    front_tab = rrb.Horizontal(
        _camera_view(_grayscale_origin("front_right"), "front right"),
        _camera_view(_grayscale_origin("front_left"), "front left"),
        _camera_view(_depth_origin("front_right"), "front right (depth)"),
        _camera_view(_depth_origin("front_left"), "front left (depth)"),
        name="front",
    )

    grayscale_tab = rrb.Vertical(
        rrb.Horizontal(
            _camera_view(_grayscale_origin("front_right"), "front right"),
            _camera_view(_grayscale_origin("front_left"), "front left"),
        ),
        rrb.Horizontal(
            _camera_view(_grayscale_origin("left"), "left"),
            _camera_view(_grayscale_origin("right"), "right"),
        ),
        _camera_view(_grayscale_origin("back"), "back"),
        name="grayscale",
    )

    depth_tab = rrb.Vertical(
        rrb.Horizontal(
            _camera_view(_depth_origin("front_right"), "front right (depth)"),
            _camera_view(_depth_origin("front_left"), "front left (depth)"),
        ),
        rrb.Horizontal(
            _camera_view(_depth_origin("left"), "left"),
            _camera_view(_depth_origin("right"), "right"),
        ),
        _camera_view(_depth_origin("back"), "back"),
        name="depth",
    )

    return rrb.Blueprint(
        rrb.Horizontal(
            world_view,
            rrb.Tabs(front_tab, grayscale_tab, depth_tab),
            column_shares=[2, 1],
        ),
        collapse_panels=True,
    )


# This is a correction to depth jitter
# Depth starts in the right location then immediately moves slightly to the wrong location because of the odom updates
# Odom is faster than depth hz, but depth is anchored dynamically to odom
# The solution is to anchor the image to the frame *at a specific time*
# Probably hasn't been noticed earlier because its less obvious for color image and for 60/30zh
# we should fix this in our rerun bridge but I'm not going to do that in this spot PR
class _ImageBakedIntoAnchor:
    def __init__(self, origin: str) -> None:
        self.origin = origin

    def __call__(self, image: Any) -> RerunData | None:
        anchor = _anchor_frame()
        if anchor is None:
            return None
        pose = _tf_buffer.get(anchor, image.frame_id, image.ts)
        if pose is None:
            return None
        import rerun as rr

        return [
            (
                self.origin,
                rr.Transform3D(
                    translation=[pose.translation.x, pose.translation.y, pose.translation.z],
                    rotation=pose.rotation.to_rerun(),
                    parent_frame=f"tf#/{anchor}",
                ),
            ),
            (_camera_entity(self.origin), image.to_rerun()),
        ]
