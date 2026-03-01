import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "detection2d.base": [
            "Detection2D",
            "Filter2D",
        ],
        "detection2d.bbox": [
            "Detection2DBBox",
        ],
        "detection2d.person": [
            "Detection2DPerson",
        ],
        "detection2d.point": [
            "Detection2DPoint",
        ],
        "detection2d.imageDetections2D": [
            "ImageDetections2D",
        ],
        "detection3d": [
            "Detection3D",
            "Detection3DBBox",
            "Detection3DPC",
            "ImageDetections3DPC",
            "PointCloudFilter",
            "height_filter",
            "radius_outlier",
            "raycast",
            "statistical",
        ],
        "imageDetections": ["ImageDetections"],
        "utils": ["TableStr"],
    },
)
