"""Factory helpers for attaching perception modules to a robot.

These helpers **lazily import** heavy perception models so that the core
`dimos.robot` package remains lightweight and CPU-friendly.  Each helper
returns a callable that can be supplied to `UnitreeGo2` via the
`perception_modules` constructor argument or the
`UnitreeGo2.attach_perception_module` method.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict

if TYPE_CHECKING:
    from dimos.robot.unitree.unitree_go2 import UnitreeGo2

__all__ = [
    "person_tracking_module",
    "object_tracking_module",
]


def person_tracking_module(robot: "UnitreeGo2") -> Dict[str, object]:
    """Attach a person-tracking stream to *robot*.

    This factory is completely optional; it is only imported/executed when
    explicitly requested by the application, keeping heavyweight ML
    dependencies out of the default execution path.
    """

    # Lazy import to avoid global dependency at startup
    from dimos.perception.person_tracker import PersonTrackingStream  # noqa: WPS433

    tracker = PersonTrackingStream(
        camera_intrinsics=robot.camera_intrinsics,
        camera_pitch=robot.camera_pitch,
        camera_height=robot.camera_height,
    )
    stream = tracker.create_stream(robot.video_stream or robot.get_ros_video_stream())

    # Expose both the raw tracker instance and the observable stream so that
    # downstream code can access whichever is required.
    robot.person_tracker = tracker  # type: ignore[attr-defined]
    robot.person_tracking_stream = stream  # type: ignore[attr-defined]

    return {"person_tracking": stream}


def object_tracking_module(robot: "UnitreeGo2") -> Dict[str, object]:
    """Attach a generic object-tracking stream to *robot*."""

    # Lazy import
    from dimos.perception.object_tracker import ObjectTrackingStream  # noqa: WPS433

    tracker = ObjectTrackingStream(
        camera_intrinsics=robot.camera_intrinsics,
        camera_pitch=robot.camera_pitch,
        camera_height=robot.camera_height,
    )
    stream = tracker.create_stream(robot.video_stream or robot.get_ros_video_stream())

    robot.object_tracker = tracker  # type: ignore[attr-defined]
    robot.object_tracking_stream = stream  # type: ignore[attr-defined]

    return {"object_tracking": stream}