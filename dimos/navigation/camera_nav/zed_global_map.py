"""ZED Mini → stable world-aligned obstacle map.

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

VOXEL = 0.08
MIN_D = 0.3
MAX_D = 8.0

zed = sl.Camera()
ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.VGA
ip.camera_fps = 15
ip.depth_mode = sl.DEPTH_MODE.NEURAL
ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
ip.coordinate_units = sl.UNIT.METER
ip.depth_maximum_distance = MAX_D

err = zed.open(ip)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"failed: {err}")
    exit(1)
print("camera open")

# Rerun MUST be spawned before enabling positional tracking.
# enable_positional_tracking() starts background threads; rr.init(spawn=True)
# calls fork() which is unsafe after threads exist on macOS → segfault.
rr.init("zed_map", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("rerun ready")

tp = sl.PositionalTrackingParameters()
tp.enable_imu_fusion = True
err = zed.enable_positional_tracking(tp)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"tracking failed: {err}")
    exit(1)
print("tracking enabled")

rt = sl.RuntimeParameters()
pc = sl.Mat()
pose = sl.Pose()
all_vkeys = np.empty((0, 3), dtype=np.int32)
frame = 0
t0 = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        if zed.get_position(pose, sl.REFERENCE_FRAME.WORLD) != sl.POSITIONAL_TRACKING_STATE.OK:
            frame += 1
            continue

        T = pose.pose_data.get_data().astype(np.float32)
        R, t = T[:3, :3], T[:3, 3]

        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        flat = pc.get_data().reshape(-1, 4)

        dist = np.linalg.norm(flat[:, :3], axis=1)
        mask = np.isfinite(flat[:, 0]) & (dist > MIN_D) & (dist < MAX_D)
        pts = flat[mask]

        if len(pts) == 0:
            frame += 1
            continue

        xyz = pts[:, :3].astype(np.float32) @ R.T + t

        vkeys = np.unique(np.floor(xyz / VOXEL).astype(np.int32), axis=0)
        if len(all_vkeys) == 0:
            all_vkeys = vkeys
        else:
            all_vkeys = np.unique(np.vstack([all_vkeys, vkeys]), axis=0)

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(pts)}  voxels={len(all_vkeys)}  fps={fps:.1f}")

        rgba = pts[:, 3].view(np.uint32)
        r = ((rgba >> 16) & 0xFF).astype(np.uint8)
        g = ((rgba >>  8) & 0xFF).astype(np.uint8)
        b = ( rgba        & 0xFF).astype(np.uint8)
        n = min(len(xyz), 20_000)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(len(xyz))
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=np.column_stack([r[idx], g[idx], b[idx]])))

        if frame % 5 == 0:
            centres = (all_vkeys.astype(np.float32) + 0.5) * VOXEL
            rr.log("world/map", rr.Points3D(positions=centres, radii=VOXEL * 0.4))

        frame += 1

except KeyboardInterrupt:
    pass

zed.disable_positional_tracking()
zed.close()
print(f"done — {len(all_vkeys)} voxels")
