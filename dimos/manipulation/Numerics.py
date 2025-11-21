class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Pose:
    def __init__(self):
        self.position = Vector3(0, 0, 0)
        self.orientation = Quaternion(0, 0, 0, 1)

class PoseStamped(Pose, Timestamped):
    msg_name = "geometry_msgs.PoseStamped"
    ts: float
    frame_id: str

    @dispatch
    def __init__(self, ts: float = 0.0, frame_id: str = "", **kwargs) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        super().__init__(**kwargs)

    def lcm_encode(self) -> bytes:
        lcm_mgs = LCMPoseStamped()
        lcm_mgs.pose = self
        [lcm_mgs.header.stamp.sec, lcm_mgs.header.stamp.nsec] = sec_nsec(self.ts)
        lcm_mgs.header.frame_id = self.frame_id
        return lcm_mgs.lcm_encode()
