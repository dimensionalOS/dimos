"""Rerun visualization helpers for M20 navigation blueprints."""

from __future__ import annotations

from typing import Any


def m20_rerun_blueprint() -> Any:
    """3D world view stacked over a 2D camera panel for the M20."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="3D"),
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            row_shares=[2, 1],
        ),
    )


def camera_info_override(camera_info: Any) -> Any:
    """Route camera intrinsics to the color_image topic for 3D projection."""
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def static_robot(rr: Any) -> list[Any]:
    """Wireframe bounding box at M20 dimensions (820x430x570mm)."""
    return [
        rr.Boxes3D(
            half_sizes=[0.41, 0.215, 0.285],
            colors=[(255, 140, 0)],
            fill_mode="wireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]
