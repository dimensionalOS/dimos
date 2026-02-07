import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "detectors": ["Detector", "Yolo2DDetector"],
        "module2D": ["Detection2DModule"],
        "module3D": ["Detection3DModule"],
    },
)
