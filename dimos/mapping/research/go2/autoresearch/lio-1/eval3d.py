# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""3D criteria for the LIO autoresearch eval, scored from an odometry trajectory
against a recording's annotations.json. Three complementary signals:

  C1 tag_spread  — detect AprilTags (36h11, 10 cm) in the recording, project each
                   to the world via the INPUT trajectory's pose, group same-id
                   detections into per-visit tracks (>15 s gap = new visit), and
                   measure the spread of a tag's tracks across visits. A drift-free
                   odom puts the same physical tag at the same world point every
                   visit → spread → 0. (The mentor's TOTAL_SPREAD idea.)
  C2 z_floor     — per floor-occupancy window, mean |z - floor_level| (z anchored
                   to the first known floor). Penalizes off-level AND non-flat.
  C3 z_ramp      — per stair transition, |net Δz - (h_to - h_from)| + a monotonicity
                   penalty. A bump-and-reset or random z scores badly.

Settings below are the Go2-L1 current settings (720p intrinsics scaled to the
recorded 1080p; equidistant/fisheye distortion; base->cam extrinsic chain).
"""
from __future__ import annotations

import struct
from itertools import combinations

import numpy as np

# --- current settings -------------------------------------------------------
TAG_SIZE_M = 0.10
_S = 1.5  # recorded frames are 1080p = 1.5x the 720p calibration
K = np.array([[797.4756 * _S, 0, 643.5352 * _S],
              [0, 796.4872 * _S, 349.2784 * _S],
              [0, 0, 1.0]])
DIST = np.array([-0.07309428880537933, -0.02341140740909078,
                 -0.0069305931780026956, 0.009238684474464793])  # fisheye k1..k4
ARUCO_DICT = "DICT_APRILTAG_36h11"
R_OPT2LINK = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])  # camera_optical -> camera_link
T_LINK2BASE = np.array([0.3, 0.0, 0.0])                        # camera_link  -> base_link
VISIT_GAP_S = 15.0   # time gap that splits a tag's detections into separate visits
TRIM_S = 3.0         # trim each floor window's edges (annotation times are approximate)

# --- minimal CDR (matches go2-station/scripts/go2_cdr.py) -------------------
class _Cur:
    def __init__(self, b):
        self.b = b; self.p = 4  # skip 4-byte encapsulation header

    def _al(self, n):
        m = (self.p - 4) % n
        if m:
            self.p += n - m

    def i32(self):
        self._al(4); v = struct.unpack_from("<i", self.b, self.p)[0]; self.p += 4; return v

    def u32(self):
        self._al(4); v = struct.unpack_from("<I", self.b, self.p)[0]; self.p += 4; return v

    def f64n(self, n):
        self._al(8); v = struct.unpack_from("<%dd" % n, self.b, self.p); self.p += 8 * n; return list(v)

    def s(self):
        n = self.u32(); self.p += n  # skip a string

    def stamp_ns(self):
        sec = self.i32(); nsec = self.u32(); return sec * 1_000_000_000 + nsec


def _decode_jpeg_bytes(data):  # sensor_msgs/CompressedImage
    c = _Cur(data); c.stamp_ns(); c.s(); c.s()  # header.stamp, frame_id, format
    n = c.u32(); return bytes(data[c.p:c.p + n])


def _decode_odom(data):  # nav_msgs/Odometry -> (pos[3], quat xyzw[4])
    c = _Cur(data); c.stamp_ns(); c.s(); c.s()
    pos = c.f64n(3); quat = c.f64n(4); return pos, quat


# --- pose helpers -----------------------------------------------------------
def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def euler_deg_to_R(rpy_deg):
    """ZYX (yaw·pitch·roll) from mat_out's SO3ToEuler (degrees, pitch=asin(2(wy-zx)))."""
    r, p, y = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# --- trajectory loaders -----------------------------------------------------
def load_mat_out(path):
    """Point-LIO Log/mat_out.txt -> (t_rel[N], pos[N,3], R[N,3,3]). Harness output."""
    M = np.loadtxt(path)
    t, eul, pos = M[:, 0], M[:, 1:4], M[:, 4:7]
    R = np.array([euler_deg_to_R(e) for e in eul])
    return t, pos, R


def load_robot_odom(mcap_path):
    """robot_odom from the recording -> (t_rel, pos, R, first_lidar_pub_ns). The
    gt-ish leg-inertial backbone; handy as the reference 'input odom' for testing."""
    from mcap.reader import make_reader
    t, pos, R, flp = [], [], [], None
    with open(mcap_path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(
                topics=["rt/utlidar/robot_odom", "rt/utlidar/cloud"]):
            if ch.topic == "rt/utlidar/cloud":
                if flp is None:
                    flp = m.publish_time
            else:
                p, q = _decode_odom(m.data)
                t.append(m.publish_time); pos.append(p); R.append(quat_to_R(*q))
    t = np.array(t); o = np.argsort(t)
    return (t[o] - flp) / 1e9, np.array(pos)[o], np.array(R)[o], flp


# --- annotation helpers -----------------------------------------------------
def _anchor_z(t, z, ann):
    """Offset z so the first known-level floor window sits at its level."""
    for ph in ann["phases"]:
        lvl = ph.get("level")
        h = ann["levels"].get(lvl) if lvl else None
        if h is None:
            continue
        m = (t >= ph["t0"] + TRIM_S) & (t <= ph["t1"] - TRIM_S)
        if m.sum() > 2:
            return z + (h - z[m].mean())
    return z


def _floor_before_after(ann, idx):
    """Levels of the nearest level-phase before/after transition phase idx."""
    before = after = None
    for j in range(idx - 1, -1, -1):
        if "level" in ann["phases"][j]:
            before = ann["phases"][j]["level"]; break
    for j in range(idx + 1, len(ann["phases"])):
        if "level" in ann["phases"][j]:
            after = ann["phases"][j]["level"]; break
    return before, after


# --- C2: floor flatness/level ----------------------------------------------
def c2_z_floor(t, z, ann):
    per, errs = {}, []
    for ph in ann["phases"]:
        lvl = ph.get("level"); h = ann["levels"].get(lvl) if lvl else None
        if h is None:
            continue
        m = (t >= ph["t0"] + TRIM_S) & (t <= ph["t1"] - TRIM_S)
        if m.sum() < 3:
            continue
        e = np.abs(z[m] - h)
        per.setdefault(lvl, []).extend(e.tolist())
    summary = {k: float(np.mean(v)) for k, v in per.items()}
    allerr = [e for v in per.values() for e in v]
    return {"z_floor_err_m": float(np.mean(allerr)) if allerr else None, "z_floor_by_level": summary}


# --- C3: transition ramp ----------------------------------------------------
def c3_z_ramp(t, z, ann):
    per, errs = [], []
    for i, ph in enumerate(ann["phases"]):
        if "name" not in ph or ph["name"] not in ("ascend", "descend"):
            continue
        fr, to = _floor_before_after(ann, i)
        hf, ht = ann["levels"].get(fr), ann["levels"].get(to)
        if hf is None or ht is None:
            continue
        gap = ht - hf
        seg = (t >= ph["t0"]) & (t <= ph["t1"])
        if seg.sum() < 3:
            continue
        zs = z[seg]
        net = zs[-3:].mean() - zs[:3].mean()
        mono = float(np.mean(np.sign(np.diff(zs)) == np.sign(gap)))
        err = abs(net - gap)
        per.append({"phase": f"{fr}->{to}", "net_dz": float(net), "gap": float(gap),
                    "err_m": float(err), "monotonic_frac": mono})
        errs.append(err)
    return {"z_ramp_err_m": float(np.mean(errs)) if errs else None, "z_ramp_by_transition": per}


# --- C1: AprilTag 3D spread -------------------------------------------------
def c1_tag_spread(t_rel, pos, R, first_lidar_pub_ns, mcap_path):
    import cv2
    from mcap.reader import make_reader
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT)),
        cv2.aruco.DetectorParameters())
    s = TAG_SIZE_M
    objp = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                     [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]])
    traj_abs = first_lidar_pub_ns + (t_rel * 1e9)  # video matched on the same clock (≈; latency «drift)
    dets = {}  # id -> list of (abs_ns, world_xyz)
    with open(mcap_path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(topics=["rt/frontvideo"]):
            jpeg = _decode_jpeg_bytes(m.data)
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            corners, ids, _ = det.detectMarkers(img)
            if ids is None:
                continue
            i = int(np.clip(np.searchsorted(traj_abs, m.publish_time), 0, len(traj_abs) - 1))
            for cn, idv in zip(corners, ids.flatten()):
                und = cv2.fisheye.undistortPoints(cn.reshape(-1, 1, 2).astype(np.float64), K, DIST)
                ok, rvec, tvec = cv2.solvePnP(objp, und, np.eye(3), None,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                p_base = R_OPT2LINK @ tvec.ravel() + T_LINK2BASE
                world = R[i] @ p_base + pos[i]
                dets.setdefault(int(idv), []).append((m.publish_time, world))
    per, spreads = {}, []
    for idv, lst in dets.items():
        lst.sort()
        ts = np.array([d[0] for d in lst]); W = np.array([d[1] for d in lst])
        groups = np.split(np.arange(len(lst)), np.where(np.diff(ts) / 1e9 > VISIT_GAP_S)[0] + 1)
        cents = np.array([W[g].mean(0) for g in groups])
        sp = float(np.mean([np.linalg.norm(cents[i] - cents[j])
                            for i, j in combinations(range(len(cents)), 2)])) if len(cents) > 1 else None
        per[idv] = {"detections": len(lst), "visits": len(cents), "spread_m": sp}
        if sp is not None:
            spreads.append(sp)
    return {"tag_spread_m": float(np.mean(spreads)) if spreads else None, "tag_by_id": per}


# --- top-level --------------------------------------------------------------
def score_3d(t_rel, pos, R, first_lidar_pub_ns, mcap_path, ann, with_tags=True):
    z = _anchor_z(t_rel, pos[:, 2].copy(), ann)
    out = {}
    out.update(c2_z_floor(t_rel, z, ann))
    out.update(c3_z_ramp(t_rel, z, ann))
    if with_tags:
        try:
            out.update(c1_tag_spread(t_rel, pos, R, first_lidar_pub_ns, mcap_path))
        except Exception as e:  # cv2/mcap missing or no tags — keep z criteria usable
            out["tag_spread_m"] = None; out["tag_error"] = repr(e)
    return out


if __name__ == "__main__":
    import argparse, json
    ap = argparse.ArgumentParser(description="3D eval (tag spread + z floor + z ramp)")
    ap.add_argument("--mcap", required=True, help="recording (.mcap) for tag frames + clock")
    ap.add_argument("--ann", required=True, help="annotations.json")
    ap.add_argument("--traj", help="mat_out.txt to score; omitted -> use robot_odom from --mcap")
    ap.add_argument("--no-tags", action="store_true")
    a = ap.parse_args()
    ann = json.load(open(a.ann))
    _, _, _, flp = load_robot_odom(a.mcap) if a.traj else (None, None, None, None)
    if a.traj:
        t, pos, R = load_mat_out(a.traj)
    else:
        t, pos, R, flp = load_robot_odom(a.mcap)
    res = score_3d(t, pos, R, flp, a.mcap, ann, with_tags=not a.no_tags)
    print(json.dumps(res, indent=2))
