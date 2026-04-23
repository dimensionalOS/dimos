#!/usr/bin/env python3
"""Probe one lidar cloud from /dev/shm/drdds_bridge_lidar. Extract
per-point time doubles at offset 18, report min/max/distribution and
whether they're monotonic. Also print the SHM slot's header timestamp.

Key question: does first_t = points[0].time equal min(time), or is the
cloud not in temporal order?
"""
import struct
import mmap
import os
import sys

SHM_NAME = "/dev/shm/drdds_bridge_lidar"
LIDAR_SLOT_SIZE = 4 * 1024 * 1024
LIDAR_NUM_SLOTS = 16
SHM_HDR_SIZE = 56   # note: NOT 64; compiler packs (FASTLIO2_LOG #24)
SLOT_HDR_SIZE = 64

# Point layout (raw drdds from rsdriver RSAIRY merged mode):
#   x:float32 y:float32 z:float32 intensity:float32 ring:uint16 time:float64
# Total 26 bytes/point.
POINT_STEP = 26
TIME_OFFSET = 18


def main() -> int:
    if not os.path.exists(SHM_NAME):
        print(f"no shm at {SHM_NAME}", file=sys.stderr)
        return 2

    total = SHM_HDR_SIZE + LIDAR_NUM_SLOTS * LIDAR_SLOT_SIZE
    f = open(SHM_NAME, "rb")
    m = mmap.mmap(f.fileno(), total, prot=mmap.PROT_READ)

    # Read ShmHeader. Layout:
    #   atomic<uint64> write_idx   (8 bytes)
    #   atomic<uint64> read_idx    (8)
    #   uint32 num_slots           (4)
    #   uint32 slot_size           (4)
    #   uint32 ready               (4)
    #   char padding[28]           (28)  → 56 bytes total
    write_idx, read_idx, num_slots, slot_size, ready = struct.unpack_from(
        "<QQIII", m, 0)
    print(f"[probe] SHM header: write_idx={write_idx} read_idx={read_idx} "
          f"num_slots={num_slots} slot_size={slot_size} ready=0x{ready:08x}")

    if write_idx == 0:
        print("[probe] writer has not produced anything yet")
        return 3

    # Grab the most recently completed slot
    slot_idx = (write_idx - 1) % num_slots
    slot_off = SHM_HDR_SIZE + slot_idx * slot_size
    print(f"[probe] reading slot {slot_idx} (write_idx={write_idx})")

    # Read SlotHeader (64 bytes)
    data_size, msg_type, stamp_sec, stamp_nsec, height, width, point_step, \
        row_step = struct.unpack_from("<IIIIIIII", m, slot_off)
    print(f"[probe] slot: data_size={data_size} msg_type={msg_type} "
          f"stamp={stamp_sec}.{stamp_nsec:09d} pts={width*height} "
          f"point_step={point_step}")

    if msg_type != 0:
        print("[probe] not a PointCloud2 slot")
        return 4

    # Per-point data starts at slot_off + SLOT_HDR_SIZE
    data_off = slot_off + SLOT_HDR_SIZE
    n_pts = width * height
    if point_step != POINT_STEP:
        print(f"[probe] WARNING: point_step={point_step} != expected {POINT_STEP}")

    # Extract time doubles for every point
    times = []
    for i in range(n_pts):
        pt_off = data_off + i * point_step
        (t,) = struct.unpack_from("<d", m, pt_off + TIME_OFFSET)
        times.append(t)

    t0 = times[0]
    tmin = min(times)
    tmax = max(times)
    tlast = times[-1]

    print(f"\n[probe] Per-point time analysis ({n_pts} points):")
    print(f"  times[0]         = {t0:.9f}")
    print(f"  times[-1]        = {tlast:.9f}")
    print(f"  min              = {tmin:.9f}")
    print(f"  max              = {tmax:.9f}")
    print(f"  max-min          = {tmax - tmin:.6f}  ({(tmax-tmin)*1000:.2f} ms)")
    print(f"  times[0] - min   = {t0 - tmin:.6f}")
    print(f"  times[-1] - max  = {tlast - tmax:.6f}")
    print(f"  header.stamp - min = {(stamp_sec + stamp_nsec*1e-9) - tmin:.6f}")
    print(f"  header.stamp - max = {(stamp_sec + stamp_nsec*1e-9) - tmax:.6f}")

    # Monotonicity check
    n_desc = sum(1 for i in range(1, n_pts) if times[i] < times[i-1])
    print(f"\n  descending-step count: {n_desc} / {n_pts-1} "
          f"({100.0 * n_desc / max(n_pts-1, 1):.1f}%)")
    if n_desc == 0:
        print("  → points ARE monotonically ordered in time")
    elif t0 == tmin:
        print("  → points NOT monotonic but first point happens to be minimum time")
    else:
        print(f"  → NOT MONOTONIC: first point is {t0-tmin:.6f}s after min. "
              f"first_t heuristic in bridge is WRONG.")

    # Print a few samples spread across the cloud
    print(f"\n[probe] Sample point times:")
    for i in [0, n_pts // 4, n_pts // 2, 3 * n_pts // 4, n_pts - 1]:
        print(f"  points[{i:6d}].time = {times[i]:.9f} "
              f"(rel to t0: {times[i] - t0:+.6f}s)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
