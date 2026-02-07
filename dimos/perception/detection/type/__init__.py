import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "detection2d": [
            "Detection2D",
            "Detection2DBBox",
            "Detection2DPerson",
            "Detection2DPoint",
            "Filter2D",
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
