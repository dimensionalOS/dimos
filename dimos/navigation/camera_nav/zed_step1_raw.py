import time
import numpy as np
import pyzed.sl as sl
import rerun as rr

zed = sl.Camera()

ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.VGA
ip.camera_fps = 15
ip.depth_mode = sl.DEPTH_MODE.NEURAL
ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
ip.coordinate_units = sl.UNIT.METER

err = zed.open(ip)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"failed: {err}")
    exit(1)

rr.init("zed_step1", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
print("camera open")

rt = sl.RuntimeParameters()
pc = sl.Mat()
frame = 0
t0 = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        data = pc.get_data()
        flat = data.reshape(-1, 4)
        mask = np.isfinite(flat[:, 0])
        flat_valid = flat[mask]
        valid = len(flat_valid)
        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame}  valid_pts={valid}  fps={fps:.1f}")

        # Subsample to ~5k points before sending to Rerun
        stride = max(1, len(flat_valid) // 5000)
        sub = flat_valid[::stride]
        xyz = sub[:, :3]
        rgba = sub[:, 3].view(np.uint32)
        r = ((rgba >> 16) & 0xFF).astype(np.uint8)
        g = ((rgba >>  8) & 0xFF).astype(np.uint8)
        b = ( rgba        & 0xFF).astype(np.uint8)
        rr.log("world/raw_cloud", rr.Points3D(positions=xyz, colors=np.column_stack([r, g, b])))
        frame += 1

except KeyboardInterrupt:
    pass

zed.close()
print("done")
