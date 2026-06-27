"""ZED Mini → color-preserving global obstacle map.

Built directly on zed_step1_raw.py. Adds per-voxel RGB accumulation so the
map looks like the real scene, not coloured by height.

Each voxel stores the average colour of all points that have landed in it.
Revisiting an area reinforces existing voxels — no duplicates, no drift.

Rerun entities:
  world/raw_cloud   current frame, real RGB (same as step1)
  world/map         growing accumulated map, real RGB per voxel

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

VOXEL   = 0.05      # 5 cm — tight enough to look like real geometry
MIN_D   = 0.3
MAX_D   = 8.0
MAX_VIZ = 20_000    # cap per-frame points sent to Rerun
MAX_MAP = 400_000   # coarsen above this

# Packing: voxel index → unique int64 key (handles negative indices)
_OFF   = np.int64(10_000)   # supports ±10 000 voxels per axis (±500 m at 5 cm)
_MASK  = np.int64((1 << 15) - 1)

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = vkeys.astype(np.int64) + _OFF
    return (v[:, 0] << 30) | (v[:, 1] << 15) | v[:, 2]

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

rt  = sl.RuntimeParameters()
pc  = sl.Mat()

# Global map state
voxel       = VOXEL
map_centers = np.empty((0, 3), dtype=np.float32)   # (N, 3) XYZ of voxel centres
map_colors  = np.empty((0, 3), dtype=np.float32)   # (N, 3) avg RGB 0-255
map_counts  = np.empty(0,      dtype=np.float32)   # (N,)   how many points merged
map_packed  = np.empty(0,      dtype=np.int64)     # (N,)   packed keys for lookup

frame = 0
t0    = time.monotonic()

try:
    while True:
        # ── Grab (same as step1) ──────────────────────────────────────────────
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        data = pc.get_data()
        flat = data.reshape(-1, 4)

        dist = np.linalg.norm(flat[:, :3], axis=1)
        mask = np.isfinite(flat[:, 0]) & (dist > MIN_D) & (dist < MAX_D)
        pts  = flat[mask]

        if len(pts) == 0:
            frame += 1
            continue

        xyz  = pts[:, :3].astype(np.float32)
        rgba = pts[:, 3].view(np.uint32)
        r    = ((rgba >> 16) & 0xFF).astype(np.float32)
        g    = ((rgba >>  8) & 0xFF).astype(np.float32)
        b    = ( rgba        & 0xFF).astype(np.float32)

        # ── Current frame viz (same as step1, capped) ────────────────────────
        n   = min(len(xyz), MAX_VIZ)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(len(xyz))
        rr.log("world/raw_cloud", rr.Points3D(
            positions=xyz[idx],
            colors=np.column_stack([r[idx], g[idx], b[idx]]).astype(np.uint8),
        ))

        # ── Voxelise current frame ────────────────────────────────────────────
        pt_vkeys  = np.floor(xyz / voxel).astype(np.int32)
        pt_packed = _pack(pt_vkeys)

        # Unique voxels in this frame + average colour per voxel
        uniq_packed, inv = np.unique(pt_packed, return_inverse=True)
        n_uniq   = len(uniq_packed)
        cnt      = np.bincount(inv, minlength=n_uniq).astype(np.float32)
        r_mean   = np.bincount(inv, weights=r, minlength=n_uniq) / cnt
        g_mean   = np.bincount(inv, weights=g, minlength=n_uniq) / cnt
        b_mean   = np.bincount(inv, weights=b, minlength=n_uniq) / cnt

        # Voxel centres for this frame's unique voxels
        uniq_vkeys   = pt_vkeys[np.searchsorted(pt_packed, uniq_packed)]   # representative point per voxel
        uniq_centers = (uniq_vkeys.astype(np.float32) + 0.5) * voxel

        # ── Merge into global map ─────────────────────────────────────────────
        if len(map_packed) == 0:
            # First frame — initialise map directly
            map_packed  = uniq_packed
            map_centers = uniq_centers
            map_colors  = np.column_stack([r_mean, g_mean, b_mean])
            map_counts  = cnt.copy()
        else:
            # Find which frame voxels already exist in the map
            sorter   = np.argsort(map_packed)
            ins      = np.searchsorted(map_packed, uniq_packed, sorter=sorter)
            ins      = np.clip(ins, 0, len(map_packed) - 1)
            existing = map_packed[sorter[ins]] == uniq_packed   # True = already in map

            # Update existing voxels: running average
            ex_idx = sorter[ins[existing]]
            new_cnt = map_counts[ex_idx] + cnt[existing]
            for ch, col_mean in enumerate([r_mean, g_mean, b_mean]):
                map_colors[ex_idx, ch] = (
                    map_colors[ex_idx, ch] * map_counts[ex_idx] + col_mean[existing] * cnt[existing]
                ) / new_cnt
            map_counts[ex_idx] = new_cnt

            # Insert new voxels
            is_new = ~existing
            if is_new.any():
                map_packed  = np.concatenate([map_packed,  uniq_packed[is_new]])
                map_centers = np.vstack([map_centers, uniq_centers[is_new]])
                map_colors  = np.vstack([map_colors,  np.column_stack([r_mean[is_new], g_mean[is_new], b_mean[is_new]])])
                map_counts  = np.concatenate([map_counts, cnt[is_new]])

        # ── Coarsen if map is too large ───────────────────────────────────────
        if len(map_packed) > MAX_MAP:
            voxel *= 2
            new_vkeys  = np.floor(map_centers / voxel).astype(np.int32)
            new_packed = _pack(new_vkeys)
            uniq_new, inv_new = np.unique(new_packed, return_inverse=True)
            cnt_new = np.bincount(inv_new, weights=map_counts).astype(np.float32)
            new_colors = np.zeros((len(uniq_new), 3), dtype=np.float32)
            for ch in range(3):
                new_colors[:, ch] = np.bincount(inv_new, weights=map_colors[:, ch] * map_counts) / cnt_new
            new_centers = (new_vkeys[np.searchsorted(new_packed, uniq_new)].astype(np.float32) + 0.5) * voxel
            map_packed  = uniq_new
            map_centers = new_centers
            map_colors  = new_colors
            map_counts  = cnt_new
            print(f"[map] coarsened → {voxel:.3f} m  ({len(map_packed)} voxels)")

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(pts)}  voxels={len(map_packed)}  fps={fps:.1f}")

        # ── Accumulated map every 5 frames ────────────────────────────────────
        if frame % 5 == 0 and len(map_centers) > 0:
            rr.log("world/map", rr.Points3D(
                positions=map_centers,
                colors=map_colors.clip(0, 255).astype(np.uint8),
            ))

        frame += 1

except KeyboardInterrupt:
    pass

zed.close()
print(f"done — {len(map_packed)} voxels")
