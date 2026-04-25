#!/usr/bin/env python3
"""M20 SLAM diagnostic harness.

Captures everything we need in a single rotation test:
  - SHM cloud snapshot pre + mid + post rotation (scene feature analysis)
  - Per-point time distribution + float32 precision check
  - LCM recording of /raw_points, /airy_imu_front, /odometry for offline replay
  - Periodic CPU/thread sampling during rotation (Python stall check)
  - Correlation of imu_vs_frame_end spikes with keyframe step-jumps

Output: /tmp/diag_<timestamp>/ directory with all artifacts.
"""
import argparse
import atexit
import math
import mmap
import os
import signal
import struct
import subprocess
import sys
import threading
import time
import statistics

import lcm
from dimos_lcm.geometry_msgs.Twist import Twist
from dimos_lcm.sensor_msgs.Imu import Imu


SHM_LIDAR = "/dev/shm/drdds_bridge_lidar"
SHM_HDR = 56
SLOT_HDR = 64
NUM_SLOTS = 16
SLOT_SIZE = 4 * 1024 * 1024


# ---------------------------------------------------------------------------
# Safety: zero-flood teleop on any exit.
# ---------------------------------------------------------------------------

# Default to /tele_cmd_vel — this is what works when smartnav's CmdVelMux is up
# (M20_NAV_ENABLED=1). In NAV_ENABLED=0 mode CmdVelMux is gone entirely, so the
# harness needs to publish /cmd_vel directly to feed NavCmdPub. Override via
# --cmd_topic on the CLI; the value here is just the import-time default.
STOP_TOPIC = "/tele_cmd_vel#geometry_msgs.Twist"
_stop_armed = [True]

def _flood_zeros(reason: str) -> None:
    if not _stop_armed[0]:
        return
    _stop_armed[0] = False
    try:
        lc = lcm.LCM()
        z = Twist(); z.angular.z = 0.0
        print(f"[safety] flooding zeros ({reason})", file=sys.stderr, flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            lc.publish(STOP_TOPIC, z.lcm_encode())
            time.sleep(0.02)
        print(f"[safety] robot stopped.", file=sys.stderr, flush=True)
    except Exception as e:
        print(f"[safety] WARN: {e!r}", file=sys.stderr, flush=True)

def _install_safety() -> None:
    atexit.register(_flood_zeros, "atexit")
    signal.signal(signal.SIGINT, lambda s, f: (_flood_zeros("SIGINT"), sys.exit(128 + s)))
    signal.signal(signal.SIGTERM, lambda s, f: (_flood_zeros("SIGTERM"), sys.exit(128 + s)))


# ---------------------------------------------------------------------------
# SHM cloud snapshot
# ---------------------------------------------------------------------------

def snapshot_shm_cloud(outpath: str) -> dict:
    """Read one lidar cloud from SHM, write binary + analysis."""
    if not os.path.exists(SHM_LIDAR):
        return {"error": "no SHM"}
    f = open(SHM_LIDAR, "rb")
    m = mmap.mmap(f.fileno(), os.fstat(f.fileno()).st_size, prot=mmap.PROT_READ)
    wi, ri, ns, ss = struct.unpack_from("<QQII", m, 0)
    slot_idx = (wi - 1) % ns
    slot_off = SHM_HDR + slot_idx * ss
    data_size, msg_type, ssec, snsec, h, w, pstep, rstep = struct.unpack_from("<IIIIIIII", m, slot_off)
    stamp_s = ssec + snsec * 1e-9
    n = h * w
    data_off = slot_off + SLOT_HDR

    # Extract all xyz + time + ring
    xs = []; ys = []; zs = []; ts = []; rings = []
    for i in range(n):
        off = data_off + i * pstep
        x, y, z, inten = struct.unpack_from("<ffff", m, off)
        ring, = struct.unpack_from("<H", m, off + 16)
        t, = struct.unpack_from("<d", m, off + 18)
        xs.append(x); ys.append(y); zs.append(z); ts.append(t); rings.append(ring)

    # Write raw cloud to file (xyz + time only, not re-readable elsewhere but keeps it compact)
    with open(outpath + ".bin", "wb") as wf:
        for x, y, z, t, r in zip(xs, ys, zs, ts, rings):
            wf.write(struct.pack("<fffdH", x, y, z, t, r))

    # Compute analysis
    rng = [math.sqrt(x*x + y*y + z*z) for x, y, z in zip(xs, ys, zs)]
    rng_sorted = sorted(rng)
    def pct(p): return rng_sorted[min(int(p * (n - 1)), n - 1)]

    # Feature sparsity proxy: how many points have neighbors within 0.1m in the same ring?
    # Too slow to compute on 50k points here — skip for now, use range stats instead.

    # Per-point time precision
    t_min, t_max = min(ts), max(ts)
    t_span = t_max - t_min
    # float32 precision relative to t_span: span / 2^23 (23 mantissa bits)
    f32_precision = t_span / (2 ** 23)

    summary = {
        "n_points": n,
        "stamp_sec": stamp_s,
        "range_min": min(rng),
        "range_max": max(rng),
        "range_p10": pct(0.10),
        "range_p50": pct(0.50),
        "range_p90": pct(0.90),
        "x_min": min(xs), "x_max": max(xs),
        "y_min": min(ys), "y_max": max(ys),
        "z_min": min(zs), "z_max": max(zs),
        "close_points_lt_1m": sum(1 for r in rng if r < 1.0),
        "close_points_lt_05m": sum(1 for r in rng if r < 0.5),
        "rings_distinct": len(set(rings)),
        "rings_min": min(rings),
        "rings_max": max(rings),
        "time_min": t_min,
        "time_max": t_max,
        "time_span_s": t_span,
        "f32_precision_us": f32_precision * 1e6,
        "header_stamp_vs_time_min_s": stamp_s - t_min,
        "header_stamp_vs_time_max_s": stamp_s - t_max,
    }
    return summary


# ---------------------------------------------------------------------------
# CPU / thread sampler (Python stall detector)
# ---------------------------------------------------------------------------

def sample_proc_state(pid_file: str = "/tmp/smartnav.pid") -> dict:
    """Sample thread count + running thread count for smartnav via /proc.

    Walks /proc/<pid>/task/*/stat to classify threads. State letter at
    position 3 in stat file: R=running, D=uninterruptible sleep,
    S=sleeping (normal), T=stopped. Many R/D during a stall indicates
    GIL or kernel contention.
    """
    try:
        with open(pid_file) as f:
            pid = int(f.read().strip())
    except Exception:
        return {"error": "no pid"}
    try:
        # Build set of pids: parent + all its children (one level deep is fine;
        # smartnav + forkserver + a few worker pids).
        root = pid
        all_pids = [root]
        try:
            with open(f"/proc/{root}/task/{root}/children") as f:
                all_pids.extend(int(x) for x in f.read().split())
        except Exception:
            pass
        total_threads = 0
        running = 0
        for p in all_pids:
            task_dir = f"/proc/{p}/task"
            if not os.path.isdir(task_dir):
                continue
            for tid in os.listdir(task_dir):
                try:
                    with open(f"{task_dir}/{tid}/stat") as sf:
                        fields = sf.read().split()
                        state = fields[2]
                        total_threads += 1
                        if state in ("R", "D"):
                            running += 1
                except Exception:
                    continue
        return {"threads": total_threads, "running": running, "total_cpu": -1}
    except Exception as e:
        return {"error": str(e)}


# ---------------------------------------------------------------------------
# Main diagnostic flow
# ---------------------------------------------------------------------------

def main() -> int:
    global STOP_TOPIC  # may be overridden by --cmd_topic; safety _flood_zeros reads it.
    _install_safety()
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir",
                    default="/var/opt/robot/data/tmp/diag_" + str(int(time.time())))
    ap.add_argument("--bag", action="store_true", default=True,
                    help="record LCM bag (400-500MB; default on)")
    ap.add_argument("--no-bag", dest="bag", action="store_false",
                    help="skip LCM bag to save disk")
    ap.add_argument("--yaw", type=float, default=-0.4)
    ap.add_argument("--rotate_duration", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--cooldown", type=float, default=5.0)
    ap.add_argument("--cmd_topic", type=str, default=STOP_TOPIC,
                    help="LCM topic to publish Twist on. Default /tele_cmd_vel goes "
                         "through smartnav's CmdVelMux (only present when NAV_ENABLED=1). "
                         "Use /cmd_vel#geometry_msgs.Twist for NAV_ENABLED=0 (drives "
                         "NavCmdPub directly).")
    args = ap.parse_args()
    STOP_TOPIC = args.cmd_topic
    print(f"[diag] cmd_topic = {STOP_TOPIC}", file=sys.stderr, flush=True)

    os.makedirs(args.outdir, exist_ok=True)
    print(f"[diag] outdir = {args.outdir}", file=sys.stderr, flush=True)

    # LCM bag recording disabled by default (each bag is 400-500MB, filled
    # /var/opt/robot/data partition within 3 reps). Enable with --bag.
    logger_proc = None
    if args.bag:
        lcm_log = os.path.join(args.outdir, "session.lcm")
        logger_proc = subprocess.Popen(
            ["lcm-logger", "-f", lcm_log,
             "-c", "/raw_points#sensor_msgs.PointCloud2|"
                   "/airy_imu_front#sensor_msgs.Imu|"
                   "/odometry#nav_msgs.Odometry|"
                   "/registered_scan#sensor_msgs.PointCloud2"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
        print(f"[diag] lcm-logger pid={logger_proc.pid} → {lcm_log}", file=sys.stderr, flush=True)

    # Snapshot pre-rotation cloud
    pre_snap = snapshot_shm_cloud(os.path.join(args.outdir, "cloud_pre"))
    with open(os.path.join(args.outdir, "cloud_pre.json"), "w") as f:
        import json; json.dump(pre_snap, f, indent=2)
    print(f"[diag] pre-rotation cloud: n_points={pre_snap.get('n_points')} "
          f"range p10/p50/p90 = {pre_snap.get('range_p10', 0):.2f}/{pre_snap.get('range_p50', 0):.2f}/{pre_snap.get('range_p90', 0):.2f} m",
          file=sys.stderr, flush=True)

    # Process state samples go here
    proc_samples = []  # list of (t, sample)
    stop_sampler = threading.Event()
    def sampler_thread():
        while not stop_sampler.is_set():
            s = sample_proc_state()
            proc_samples.append((time.monotonic(), s))
            time.sleep(0.1)  # 10Hz sample
    sampler = threading.Thread(target=sampler_thread, daemon=True)
    sampler.start()

    # IMU + log timestamp capture for correlation
    lc = lcm.LCM()
    imu_samples = []  # (t, wall_t, gz, ax, ay, az)
    first_wall = []

    def imu_handler(ch, data):
        try:
            m = Imu.lcm_decode(data)
        except Exception:
            return
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        if not first_wall:
            first_wall.append(time.monotonic())
        imu_samples.append((t, time.monotonic(),
                            m.angular_velocity.z,
                            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

    lc.subscribe("/airy_imu_front#sensor_msgs.Imu", imu_handler)
    while not first_wall:
        lc.handle_timeout(200)
    t0 = first_wall[0]

    # WARMUP
    print(f"[diag] t+0.0 WARMUP", file=sys.stderr, flush=True)
    while time.monotonic() < t0 + args.warmup:
        lc.handle_timeout(20)

    # MID snapshot
    mid_snap = snapshot_shm_cloud(os.path.join(args.outdir, "cloud_mid"))
    with open(os.path.join(args.outdir, "cloud_mid.json"), "w") as f:
        import json; json.dump(mid_snap, f, indent=2)

    # ROTATION
    t_rot = time.monotonic()
    rot_end = t_rot + args.rotate_duration
    print(f"[diag] t+{t_rot-t0:.1f} ROTATE", file=sys.stderr, flush=True)
    rotate = Twist(); rotate.angular.z = args.yaw
    next_pub = t_rot
    pub_count = 0
    while time.monotonic() < rot_end:
        now = time.monotonic()
        if now >= next_pub:
            lc.publish(STOP_TOPIC, rotate.lcm_encode())
            pub_count += 1
            next_pub += 0.05
        lc.handle_timeout(5)

    # Post snapshot
    post_snap = snapshot_shm_cloud(os.path.join(args.outdir, "cloud_post"))
    with open(os.path.join(args.outdir, "cloud_post.json"), "w") as f:
        import json; json.dump(post_snap, f, indent=2)

    # COOLDOWN: keep publishing zero Twists at 20 Hz so the mux keeps
    # teleop active. If we stopped publishing, teleop_cooldown_sec=1.0
    # would expire and simple_planner's commands would reach the mux,
    # potentially re-rotating the robot toward a stale/drifted goal.
    # Robot stands in place for the whole cooldown window.
    stop_msg = Twist()
    stop_msg.angular.z = 0.0
    print(f"[diag] t+{time.monotonic()-t0:.1f} COOLDOWN (holding zero Twist for {args.cooldown}s)",
          file=sys.stderr, flush=True)
    cool_end = time.monotonic() + args.cooldown
    next_pub = time.monotonic()
    while time.monotonic() < cool_end:
        now = time.monotonic()
        if now >= next_pub:
            lc.publish(STOP_TOPIC, stop_msg.lcm_encode())
            next_pub += 0.05
        lc.handle_timeout(5)

    # Final zero flood (atexit-safe), then SIGTERM smartnav (graceful
    # shutdown → sit-down command). Robot: stands during cooldown, then
    # sits down.
    _flood_zeros("end of cooldown")

    stop_sampler.set()
    sampler.join(timeout=1.0)
    if logger_proc is not None:
        logger_proc.terminate()
        logger_proc.wait(timeout=2.0)

    # Hard-stop: kill smartnav so simple_planner can't emit nav commands
    # after we release the mux. Without this, simple_planner chases any
    # cached goal (even one with a long-drifted current-pose estimate)
    # and publishes large yaw commands that the mux forwards the moment
    # teleop cooldown expires. Killing smartnav is the safest guarantee
    # that the robot stays stopped after a diag run.
    try:
        with open("/tmp/smartnav.pid") as f:
            snpid = int(f.read().strip())
        os.kill(snpid, signal.SIGTERM)
        print(f"[diag] SIGTERM sent to smartnav pid={snpid} — robot safe.",
              file=sys.stderr, flush=True)
    except Exception as e:
        print(f"[diag] WARN: couldn't stop smartnav: {e!r}",
              file=sys.stderr, flush=True)

    # Save imu + process samples to CSV
    with open(os.path.join(args.outdir, "imu.csv"), "w") as f:
        f.write("sensor_t,wall_t,gz,ax,ay,az\n")
        for r in imu_samples:
            f.write(",".join(f"{v:.6f}" for v in r) + "\n")

    with open(os.path.join(args.outdir, "proc.csv"), "w") as f:
        f.write("wall_t,threads,running,total_cpu\n")
        for t, s in proc_samples:
            f.write(f"{t-t0:.3f},{s.get('threads',-1)},{s.get('running',-1)},{s.get('total_cpu',-1)}\n")

    # Snapshot log files
    for path in ["/tmp/smartnav_native.log", "/var/log/drdds_recv.log"]:
        if os.path.exists(path):
            dst = os.path.join(args.outdir, os.path.basename(path))
            subprocess.run(["cp", path, dst], check=False)

    # Copy keyframe trajectory out (grep from log)
    subprocess.run(
        ["bash", "-c",
         f"grep -aE 'Keyframe [0-9]+ added' /tmp/smartnav_native.log > {args.outdir}/keyframes.txt || true"],
        check=False,
    )

    print(f"[diag] done. artifacts in {args.outdir}", file=sys.stderr, flush=True)
    print(f"[diag]   pre.json  post.json  mid.json  imu.csv  proc.csv  session.lcm  keyframes.txt  smartnav_native.log",
          file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
