import time
import numpy as np
import pyzed.sl as sl

print("1: imports ok")

zed = sl.Camera()
print("2: camera created")

ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.VGA
ip.camera_fps = 15
ip.depth_mode = sl.DEPTH_MODE.PERFORMANCE
print("3: init params set")

err = zed.open(ip)
print(f"4: open result: {err}")
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# NO tracking - test grab alone first
rt = sl.RuntimeParameters()
print("5: grabbing (no tracking)")
err = zed.grab(rt)
print(f"6: grab result: {err}")

pc = sl.Mat()
zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
data = pc.get_data()
print(f"7: pc shape: {data.shape}")

zed.close()
print("8: done")
