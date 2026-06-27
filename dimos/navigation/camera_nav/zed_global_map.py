"""ZED Mini → color-preserving global obstacle map.

Built on zed_step1_raw.py. Adds per-voxel colour accumulation so the map
shows the actual scene colours, not pink defaults.

Rerun entities:
  world/raw_cloud   current frame, real RGB (same as step1)
  world/map         accumulated voxel map, real RGB per voxel

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

VOXEL   = 0.03      # 3 cm — tight enough to show real object geometry
MIN_D   = 0.3
MAX_D   = 8.0
MAX_VIZ = 20_000    # cap for current-frame Rerun log
MAX_MAP = 500_000   # coarsen above this

# ── Open ZED (identical to zed_step1_raw.py) ─────────────────────────────────
zed = sl.Camera()
ip  = sl.InitParameters()
ip.camera_resolution      = sl.RESOLUTION.VGA
ip.camera_fps             = 15
ip.depth_mode             = sl.DEPTH_MODE.NEURAL
ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
ip.coordinate_units       = sl.UNIT.METER
ip.depth_maximum_distance = MAX_D

err = zed.open(ip)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"failed: {err}")
    exit(1)

rr.init("zed_map", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("camera open")

rt = sl.RuntimeParameters()
pc = sl.Mat()

# Map state: parallel arrays — one entry per voxel
voxel      = VOXEL
map_xyz    = np.empty((0, 3), dtype=np.float32)   # voxel centre positions
map_rgb    = np.empty((0, 3), dtype=np.uint8)      # first-seen camera colour
# Packed int64 key per voxel for O(log N) existence checks
# Offset of 100 000 handles ±100 000 voxels per axis (±3 km at 3 cm)
_OFF  = np.int64(100_000)
_MASK = np.int64((1 << 18) - 1)   # 18 bits → 262 144 > 200 000
map_keys = np.empty(0, dtype=np.int64)   # sorted at all times

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = vkeys.astype(np.int64) + _OFF
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]

frame = 0
t0    = time.monotonic()

try:
    while True:
        # ── Grab + retrieve (identical to step1) ──────────────────────────────
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        flat = pc.get_data().reshape(-1, 4)

        dist = np.linalg.norm(flat[:, :3], axis=1)
        mask = np.isfinite(flat[:, 0]) & (dist > MIN_D) & (dist < MAX_D)
        pts  = flat[mask]

        if len(pts) == 0:
            frame += 1
            continue

        xyz  = pts[:, :3].astype(np.float32)
        rgba = pts[:, 3].view(np.uint32)
        r    = ((rgba >> 16) & 0xFF).astype(np.uint8)
        g    = ((rgba >>  8) & 0xFF).astype(np.uint8)
        b    = ( rgba        & 0xFF).astype(np.uint8)
        rgb  = np.column_stack([r, g, b])

        # ── Current frame (same as step1, capped) ────────────────────────────
        n   = min(len(xyz), MAX_VIZ)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(len(xyz))
        rr.log("world/raw_cloud", rr.Points3D(
            positions=xyz[idx],
            colors=rgb[idx],
            radii=0.01,
        ))

        # ── Per-frame voxelisation ────────────────────────────────────────────
        pt_vkeys  = np.floor(xyz / voxel).astype(np.int32)   # (N, 3)
        pt_keys   = _pack(pt_vkeys)                           # (N,)  int64

        # Unique voxels this frame — keep one representative point per voxel
        order     = np.argsort(pt_keys, kind="stable")
        sorted_k  = pt_keys[order]
        diff      = np.empty(len(sorted_k), dtype=bool)
        diff[0]   = True
        diff[1:]  = sorted_k[1:] != sorted_k[:-1]
        first_occ = order[diff]                    # index into pts for each unique voxel
        fr_keys   = pt_keys[first_occ]             # (M,) unique keys this frame
        fr_xyz    = (pt_vkeys[first_occ].astype(np.float32) + 0.5) * voxel   # voxel centres
        fr_rgb    = rgb[first_occ]                 # first-seen colour per voxel

        # ── Merge into global map ─────────────────────────────────────────────
        if len(map_keys) == 0:
            map_keys = np.sort(fr_keys)
            map_xyz  = fr_xyz[np.argsort(fr_keys)]
            map_rgb  = fr_rgb[np.argsort(fr_keys)]
        else:
            # Find which frame voxels are not yet in the map (sorted → searchsorted)
            ins      = np.searchsorted(map_keys, fr_keys)
            ins_clip = np.clip(ins, 0, len(map_keys) - 1)
            is_new   = map_keys[ins_clip] != fr_keys

            if is_new.any():
                new_keys = fr_keys[is_new]
                new_xyz  = fr_xyz[is_new]
                new_rgb  = fr_rgb[is_new]
                # Merge and keep sorted
                all_keys = np.concatenate([map_keys, new_keys])
                all_xyz  = np.vstack([map_xyz, new_xyz])
                all_rgb  = np.vstack([map_rgb, new_rgb])
                order2   = np.argsort(all_keys, kind="stable")
                map_keys = all_keys[order2]
                map_xyz  = all_xyz[order2]
                map_rgb  = all_rgb[order2]

        # ── Coarsen if too large ──────────────────────────────────────────────
        if len(map_keys) > MAX_MAP:
            voxel  *= 2
            new_vk  = np.floor(map_xyz / voxel).astype(np.int32)
            new_k   = _pack(new_vk)
            ord3    = np.argsort(new_k, kind="stable")
            new_k   = new_k[ord3]; new_vk = new_vk[ord3]; c_rgb = map_rgb[ord3]
            diff2   = np.empty(len(new_k), dtype=bool)
            diff2[0] = True; diff2[1:] = new_k[1:] != new_k[:-1]
            first2  = np.where(diff2)[0]
            map_keys = new_k[first2]
            map_xyz  = (new_vk[first2].astype(np.float32) + 0.5) * voxel
            map_rgb  = c_rgb[first2]
            print(f"[map] coarsened → {voxel:.3f} m  ({len(map_keys)} voxels)")

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(pts)}  voxels={len(map_keys)}  fps={fps:.1f}")

        # ── Accumulated map every 5 frames ────────────────────────────────────
        if frame % 5 == 0 and len(map_xyz) > 0:
            rr.log("world/map", rr.Points3D(
                positions=map_xyz,
                colors=map_rgb,
                radii=voxel * 0.45,   # voxels just touch — no overlap, no gaps
            ))

        frame += 1

except KeyboardInterrupt:
    pass

zed.close()
print(f"done — {len(map_keys)} voxels")
