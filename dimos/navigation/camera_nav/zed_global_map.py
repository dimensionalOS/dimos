"""ZED Mini → stable camera-frame obstacle map.

Accumulates XYZRGBA point clouds into a persistent voxel hash map.
No ZED positional tracking (crashes in SDK 5.4.0) — points stay in
camera frame. Map is stable for a stationary camera; add pose transform
once SDK tracking is fixed.

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

VOXEL     = 0.08
MIN_D     = 0.3
MAX_D     = 8.0
MAX_VIZ   = 20_000
MAX_MAP   = 500_000

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

rr.init("zed_map", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("ready — building map. Ctrl-C to quit.")

rt = sl.RuntimeParameters()
pc = sl.Mat()
voxel = VOXEL
all_vkeys = np.empty((0, 3), dtype=np.int32)
frame = 0
t0 = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        flat = pc.get_data().reshape(-1, 4)

        dist = np.linalg.norm(flat[:, :3], axis=1)
        mask = np.isfinite(flat[:, 0]) & (dist > MIN_D) & (dist < MAX_D)
        pts = flat[mask]

        if len(pts) == 0:
            frame += 1
            continue

        xyz = pts[:, :3].astype(np.float32)

        # voxel fusion — no duplicates on revisit
        vkeys = np.unique(np.floor(xyz / voxel).astype(np.int32), axis=0)
        if len(all_vkeys) == 0:
            all_vkeys = vkeys
        else:
            all_vkeys = np.unique(np.vstack([all_vkeys, vkeys]), axis=0)

        if len(all_vkeys) > MAX_MAP:
            all_vkeys = np.unique(all_vkeys >> 1, axis=0)
            voxel *= 2
            print(f"[map] coarsened → {voxel:.3f} m  ({len(all_vkeys)} voxels)")

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(pts)}  voxels={len(all_vkeys)}  fps={fps:.1f}")

        # current frame — cap points sent to Rerun
        rgba = pts[:, 3].view(np.uint32)
        r = ((rgba >> 16) & 0xFF).astype(np.uint8)
        g = ((rgba >>  8) & 0xFF).astype(np.uint8)
        b = ( rgba        & 0xFF).astype(np.uint8)
        n = min(len(xyz), MAX_VIZ)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(len(xyz))
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=np.column_stack([r[idx], g[idx], b[idx]])))

        # accumulated map every 5 frames
        if frame % 5 == 0:
            centres = (all_vkeys.astype(np.float32) + 0.5) * voxel
            rr.log("world/map", rr.Points3D(positions=centres, radii=voxel * 0.4))

        frame += 1

except KeyboardInterrupt:
    pass

zed.close()
print(f"done — {len(all_vkeys)} voxels")
