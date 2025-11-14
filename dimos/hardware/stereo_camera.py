from dimos.hardware.camera import Camera

class StereoCamera(Camera):
    def __init__(self, baseline=None, **kwargs):
        super().__init__(**kwargs)
        self.baseline = baseline

    def get_intrinsics(self):
        intrinsics = super().get_intrinsics()
        intrinsics['baseline'] = self.baseline
        return intrinsics
