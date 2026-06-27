"""ZED Mini → dense photorealistic point cloud + accumulated map.

Built on zed_step1_raw.py. Uses sl.VIEW.LEFT for actual camera RGB so
colours match reality. Each frame produces a dense 3-D projection of the
scene (like FoundationStereo), plus an accumulated voxel map.

Rerun entities
  world/frame        dense current-frame cloud, real camera RGB
  world/depth        same geometry, depth-heatmap colours (jet: red=close)
  world/map          accumulated voxel map, real RGB per voxel

Usage:
    python -m dimos.navigation.camera_nav.zed_global_map
"""
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

STRIDE  = 2          # pixel stride → ~63 k pts at VGA, safe for Rerun
MIN_D   = 0.3
MAX_D   = 8.0
VOXEL   = 0.03       # 3 cm accumulation grid
MAX_MAP = 500_000

# ── Jet colour map (0 = close = warm red, 1 = far = cool blue) ───────────────
def _jet(t: np.ndarray) -> np.ndarray:
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)

# ── Voxel key packing (int64, handles ±100 000 voxels per axis ≈ ±3 km) ──────
_OFF   = np.int64(100_000)
_M18   = np.int64(0x3FFFF)   # 18-bit mask

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _OFF) & _M18
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]

# ── Open ZED (same as zed_step1_raw.py) ──────────────────────────────────────
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

rr.init("zed_dense", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("camera open")

rt      = sl.RuntimeParameters()
pc_mat  = sl.Mat()
img_mat = sl.Mat()

voxel    = VOXEL
map_keys = np.empty(0, dtype=np.int64)       # kept sorted at all times
map_xyz  = np.empty((0, 3), dtype=np.float32)
map_rgb  = np.empty((0, 3), dtype=np.uint8)

frame = 0
t0    = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        # ── Left camera image → actual RGB (BGRA from SDK → flip to RGB) ─────
        zed.retrieve_image(img_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        bgra    = img_mat.get_data()                        # H×W×4  uint8  BGRA
        cam_rgb = bgra[:, :, 2::-1].copy()                 # H×W×3  uint8  RGB

        # ── Point cloud → XYZ per pixel ───────────────────────────────────────
        zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        xyz_hw = pc_mat.get_data()[:, :, :3]               # H×W×3  float32

        # ── Stride subsample (regular grid → dense, not sparse random) ────────
        xyz_s = xyz_hw[::STRIDE, ::STRIDE, :].reshape(-1, 3).astype(np.float32)
        rgb_s = cam_rgb[::STRIDE, ::STRIDE, :].reshape(-1, 3)

        dist = np.linalg.norm(xyz_s, axis=1)
        ok   = np.isfinite(xyz_s[:, 0]) & (dist > MIN_D) & (dist < MAX_D)
        xyz  = xyz_s[ok]
        rgb  = rgb_s[ok]
        d    = dist[ok]

        if len(xyz) == 0:
            frame += 1
            continue

        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  pts={len(xyz)}  voxels={len(map_keys)}  fps={fps:.1f}")

        # ── Dense current-frame cloud — real camera RGB ────────────────────────
        rr.log("world/frame", rr.Points3D(
            positions=xyz,
            colors=rgb,
            radii=0.005,   # 5 mm — tight, shows real surface detail
        ))

        # ── Same geometry coloured by depth (jet heatmap) ─────────────────────
        t_norm = np.clip((d - MIN_D) / (MAX_D - MIN_D), 0, 1)
        rr.log("world/depth", rr.Points3D(
            positions=xyz,
            colors=_jet(t_norm),
            radii=0.005,
        ))

        # ── Voxel accumulation ────────────────────────────────────────────────
        pt_vk  = np.floor(xyz / voxel).astype(np.int32)
        pt_k   = _pack(pt_vk)

        # Unique voxels this frame (stable argsort → first occurrence per voxel)
        ord0   = np.argsort(pt_k, kind="stable")
        sk     = pt_k[ord0]
        diff   = np.empty(len(sk), dtype=bool)
        diff[0] = True; diff[1:] = sk[1:] != sk[:-1]
        fi     = ord0[diff]                            # first occurrence index
        fr_k   = pt_k[fi]
        fr_xyz = (pt_vk[fi].astype(np.float32) + 0.5) * voxel
        fr_rgb = rgb[fi]

        # Merge new voxels into sorted global map
        if len(map_keys) == 0:
            srt      = np.argsort(fr_k, kind="stable")
            map_keys = fr_k[srt]; map_xyz = fr_xyz[srt]; map_rgb = fr_rgb[srt]
        else:
            ins   = np.searchsorted(map_keys, fr_k)
            ins_c = np.clip(ins, 0, len(map_keys) - 1)
            new   = map_keys[ins_c] != fr_k

            if new.any():
                ak = np.concatenate([map_keys, fr_k[new]])
                ax = np.vstack([map_xyz, fr_xyz[new]])
                ac = np.vstack([map_rgb, fr_rgb[new]])
                o  = np.argsort(ak, kind="stable")
                map_keys = ak[o]; map_xyz = ax[o]; map_rgb = ac[o]

        # Coarsen if map is too large
        if len(map_keys) > MAX_MAP:
            voxel   *= 2
            new_vk   = np.floor(map_xyz / voxel).astype(np.int32)
            new_k    = _pack(new_vk)
            o2       = np.argsort(new_k, kind="stable")
            new_k    = new_k[o2]; new_vk = new_vk[o2]; cr = map_rgb[o2]
            d2       = np.empty(len(new_k), dtype=bool)
            d2[0]    = True; d2[1:] = new_k[1:] != new_k[:-1]
            f2       = np.where(d2)[0]
            map_keys = new_k[f2]
            map_xyz  = (new_vk[f2].astype(np.float32) + 0.5) * voxel
            map_rgb  = cr[f2]
            print(f"[map] coarsened → {voxel:.3f} m  ({len(map_keys)} voxels)")

        # Accumulated map every 5 frames
        if frame % 5 == 0 and len(map_xyz) > 0:
            rr.log("world/map", rr.Points3D(
                positions=map_xyz,
                colors=map_rgb,
                radii=voxel * 0.45,
            ))

        frame += 1

except KeyboardInterrupt:
    pass

zed.close()
print(f"done — {len(map_keys)} voxels")
