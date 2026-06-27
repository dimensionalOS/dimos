"""ZED Mini → stable world-aligned obstacle map.

Each frame's XYZRGBA cloud is transformed into the world frame using ZED VIO pose,
then fused into a persistent voxel hash map keyed on discretised (x,y,z).
New voxels are inserted; revisited voxels are reinforced — no duplicates.

Rerun entities:
  world/cloud   current frame RGB (updates every frame)
  world/map     growing voxel map, height-coloured (updates every 5 frames)

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""

import time

import numpy as np
import pyzed.sl as sl
import rerun as rr

VOXEL     = 0.08    # 8 cm voxel grid
MIN_DEPTH = 0.30
MAX_DEPTH = 8.00
MAX_VOXELS = 500_000  # coarsen above this threshold


def _height_colors(z: np.ndarray) -> np.ndarray:
    c = np.empty((len(z), 3), dtype=np.uint8)
    c[:] = [210, 60, 60]          # obstacles: red
    c[z < 0.25]  = [60, 200, 80]  # floor:     green
    c[z > 1.80]  = [60, 120, 220] # overhead:  blue
    return c


# ── Open ZED ─────────────────────────────────────────────────────────────────
zed = sl.Camera()
ip  = sl.InitParameters()
ip.camera_resolution      = sl.RESOLUTION.VGA
ip.camera_fps             = 15
ip.depth_mode             = sl.DEPTH_MODE.NEURAL
ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
ip.coordinate_units       = sl.UNIT.METER
ip.depth_maximum_distance = MAX_DEPTH

err = zed.open(ip)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"ZED open failed: {err}")
    exit(1)

# Minimal tracking params — avoid any SDK 5.x attribute-name pitfalls
tp = sl.PositionalTrackingParameters()
tp.enable_imu_fusion = True
err = zed.enable_positional_tracking(tp)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"positional tracking failed: {err}")
    zed.close()
    exit(1)
print("tracking enabled")

# ── Rerun ─────────────────────────────────────────────────────────────────────
rr.init("zed_global_map", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("camera open — building global map. Ctrl-C to quit.")

# ── Loop state ────────────────────────────────────────────────────────────────
rt    = sl.RuntimeParameters()
pc    = sl.Mat()
pose  = sl.Pose()
voxel = VOXEL
all_vkeys: np.ndarray = np.empty((0, 3), dtype=np.int32)

frame = 0
t0    = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        # ── Pose: world ← camera ─────────────────────────────────────────────
        state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            frame += 1
            continue

        # 4x4 homogeneous transform (row-major flat list of 16 floats)
        T = np.array(pose.pose_data.m, dtype=np.float32).reshape(4, 4)
        R = T[:3, :3]
        t = T[:3, 3]

        # ── Point cloud ───────────────────────────────────────────────────────
        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        data = pc.get_data()
        flat = data.reshape(-1, 4)

        dist = np.linalg.norm(flat[:, :3], axis=1)
        mask = np.isfinite(flat[:, 0]) & (dist > MIN_DEPTH) & (dist < MAX_DEPTH)
        pts  = flat[mask]

        if len(pts) == 0:
            frame += 1
            continue

        # ── Transform to world frame ──────────────────────────────────────────
        xyz_world = pts[:, :3].astype(np.float32) @ R.T + t

        # ── Decode colours ────────────────────────────────────────────────────
        rgba = pts[:, 3].view(np.uint32)
        r_ch = ((rgba >> 16) & 0xFF).astype(np.uint8)
        g_ch = ((rgba >>  8) & 0xFF).astype(np.uint8)
        b_ch = ( rgba        & 0xFF).astype(np.uint8)

        # ── Voxel fusion — vectorised, no Python loops ────────────────────────
        frame_vkeys = np.floor(xyz_world / voxel).astype(np.int32)
        frame_vkeys = np.unique(frame_vkeys, axis=0)   # deduplicate within frame

        if len(all_vkeys) == 0:
            all_vkeys = frame_vkeys
        else:
            all_vkeys = np.unique(np.vstack([all_vkeys, frame_vkeys]), axis=0)

        # Coarsen if map exceeds cap
        if len(all_vkeys) > MAX_VOXELS:
            all_vkeys = np.unique(all_vkeys >> 1, axis=0)
            voxel *= 2
            print(f"[map] coarsened → {voxel:.3f} m  ({len(all_vkeys)} voxels)")

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(pts)}  voxels={len(all_vkeys)}  fps={fps:.1f}")

        # ── Rerun: current frame (every frame) ───────────────────────────────
        rr.log("world/cloud", rr.Points3D(
            positions=xyz_world,
            colors=np.column_stack([r_ch, g_ch, b_ch]),
        ))

        # ── Rerun: global map (every 5 frames) ────────────────────────────────
        if frame % 5 == 0:
            centres = (all_vkeys.astype(np.float32) + 0.5) * voxel
            rr.log("world/map", rr.Points3D(
                positions=centres,
                colors=_height_colors(centres[:, 2]),
                radii=voxel * 0.4,
            ))

        frame += 1

except KeyboardInterrupt:
    pass

zed.disable_positional_tracking()
zed.close()
print(f"done — {len(all_vkeys)} voxels in final map")
