import time
import numpy as np
import pyzed.sl as sl

zed = sl.Camera()

ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.VGA
ip.camera_fps = 15
ip.depth_mode = sl.DEPTH_MODE.NEURAL
ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
ip.coordinate_units = sl.UNIT.METER

err = zed.open(ip)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"failed to open: {err}")
    exit(1)

tp = sl.PositionalTrackingParameters()
tp.enable_imu_fusion = True
tp.set_floor_as_origin = True
zed.enable_positional_tracking(tp)

print("camera open, tracking enabled")

rt = sl.RuntimeParameters()
pc = sl.Mat()
frame = 0
t0 = time.monotonic()

try:
    while True:
        if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA)
        data = pc.get_data()
        n_total = data.shape[0] * data.shape[1]
        n_valid = int(np.isfinite(data[:, :, 0]).sum())
        fps = frame / max(time.monotonic() - t0, 1e-6)
        print(f"frame={frame} total={n_total} valid={n_valid} fps={fps:.1f}")
        frame += 1
except KeyboardInterrupt:
    pass

zed.close()
