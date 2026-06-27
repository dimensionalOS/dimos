"""Step 1: raw XYZRGBA — print stats only, no Rerun, no processing."""
import time
import numpy as np
import pyzed.sl as sl


def main() -> None:
    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.depth_mode        = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units  = sl.UNIT.METER
    ip.camera_fps        = 15
    ip.camera_resolution = sl.RESOLUTION.VGA

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED failed to open")

    tp = sl.PositionalTrackingParameters()
    tp.enable_imu_fusion   = True
    tp.set_floor_as_origin = True
    zed.enable_positional_tracking(tp)

    rt     = sl.RuntimeParameters()
    pc_mat = sl.Mat()
    frame  = 0
    t_prev = time.monotonic()

    print("frame  | valid_pts | fps  | tracking")
    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            pts = pc_mat.get_data().reshape(-1, 4)
            valid = np.sum(np.isfinite(pts[:, 0]))

            pose  = sl.Pose()
            state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            ok    = "WORLD" if state == sl.POSITIONAL_TRACKING_STATE.OK else "INIT"

            t_now  = time.monotonic()
            fps    = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now

            print(f"{frame:5d}  | {valid:9d} | {fps:4.1f} | {ok}")
            frame += 1

    except KeyboardInterrupt:
        pass
    finally:
        zed.close()


if __name__ == "__main__":
    main()
