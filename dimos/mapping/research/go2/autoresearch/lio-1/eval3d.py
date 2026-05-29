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

from itertools import combinations
import json
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# --- current settings -------------------------------------------------------
TAG_SIZE_M = 0.10
_S = 1.5  # recorded frames are 1080p = 1.5x the 720p calibration
K = np.array([[797.4756 * _S, 0, 643.5352 * _S], [0, 796.4872 * _S, 349.2784 * _S], [0, 0, 1.0]])
DIST = np.array(
    [-0.07309428880537933, -0.02341140740909078, -0.0069305931780026956, 0.009238684474464793]
)  # fisheye k1..k4
ARUCO_DICT = "DICT_APRILTAG_36h11"
R_OPT2LINK = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])  # camera_optical -> camera_link
T_LINK2BASE = np.array([0.3, 0.0, 0.0])  # camera_link  -> base_link
VISIT_GAP_S = 15.0  # time gap that splits a tag's detections into separate visits
TRIM_S = 3.0  # trim each floor window's edges (annotation times are approximate)

# Total |Δz| the robot really travels in a recording = Σ |floor-gap| over its stair
# transitions, from the tape-measured floor levels. Hardcoded per dataset on purpose
# (do not read annotations for it). Keyed by a substring of the mcap path.
Z_TRAVEL_M = {
    "go2dds_data2": 2.090,  # 2.5F<->2F, descend + ascend (1.045 x2)
    "go2dds_data3": 23.090,  # 1.045 x2 + 3.5 x6 across 1F / 2F / 2.5F / 3F
}


# --- minimal CDR (matches go2-station/scripts/go2_cdr.py) -------------------
class _Cur:
    def __init__(self, b):
        self.b = b
        self.p = 4  # skip 4-byte encapsulation header

    def _al(self, n):
        m = (self.p - 4) % n
        if m:
            self.p += n - m

    def i32(self):
        self._al(4)
        v = struct.unpack_from("<i", self.b, self.p)[0]
        self.p += 4
        return v

    def u32(self):
        self._al(4)
        v = struct.unpack_from("<I", self.b, self.p)[0]
        self.p += 4
        return v

    def f64n(self, n):
        self._al(8)
        v = struct.unpack_from("<%dd" % n, self.b, self.p)
        self.p += 8 * n
        return list(v)

    def s(self):
        n = self.u32()
        self.p += n  # skip a string

    def stamp_ns(self):
        sec = self.i32()
        nsec = self.u32()
        return sec * 1_000_000_000 + nsec


def _decode_jpeg_bytes(data):  # sensor_msgs/CompressedImage
    c = _Cur(data)
    c.stamp_ns()
    c.s()
    c.s()  # header.stamp, frame_id, format
    n = c.u32()
    return bytes(data[c.p : c.p + n])


def _decode_odom(data):  # nav_msgs/Odometry -> (pos[3], quat xyzw[4])
    c = _Cur(data)
    c.stamp_ns()
    c.s()
    c.s()
    pos = c.f64n(3)
    quat = c.f64n(4)
    return pos, quat


# --- pose helpers -----------------------------------------------------------
def quat_to_R(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def euler_deg_to_R(rpy_deg):
    """ZYX (yaw·pitch·roll) from mat_out's SO3ToEuler (degrees, pitch=asin(2(wy-zx))).
    Vectorized: rpy_deg (3,) -> (3,3); (N,3) -> (N,3,3)."""
    a = np.radians(np.atleast_2d(np.asarray(rpy_deg, float)))
    r, p, y = a[:, 0], a[:, 1], a[:, 2]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R[0] if np.ndim(rpy_deg) == 1 else R


# --- trajectory loaders -----------------------------------------------------
def load_mat_out(path):
    """Point-LIO Log/mat_out.txt -> (t_rel[N], pos[N,3], R[N,3,3]). Harness output."""
    M = np.loadtxt(path, usecols=(0, 1, 2, 3, 4, 5, 6))  # t, euler(rpy deg), pos(xyz) — skip the rest
    t, eul, pos = M[:, 0], M[:, 1:4], M[:, 4:7]
    R = euler_deg_to_R(eul)  # vectorized -> (N,3,3)
    return t, pos, R


def load_robot_odom(mcap_path):
    """robot_odom from the recording -> (t_rel, pos, R, first_lidar_pub_ns). The
    gt-ish leg-inertial backbone; handy as the reference 'input odom' for testing."""
    from mcap.reader import make_reader

    t, pos, R, flp = [], [], [], None
    with open(mcap_path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(
            topics=["rt/utlidar/robot_odom", "rt/utlidar/cloud"]
        ):
            if ch.topic == "rt/utlidar/cloud":
                if flp is None:
                    flp = m.publish_time
            else:
                p, q = _decode_odom(m.data)
                t.append(m.publish_time)
                pos.append(p)
                R.append(quat_to_R(*q))
    t = np.array(t)
    o = np.argsort(t)
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
            before = ann["phases"][j]["level"]
            break
    for j in range(idx + 1, len(ann["phases"])):
        if "level" in ann["phases"][j]:
            after = ann["phases"][j]["level"]
            break
    return before, after


def _path_len(P):
    P = np.asarray(P)
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))


def _dataset_key(mcap_path):
    s = str(mcap_path)
    for k in Z_TRAVEL_M:
        if k in s:
            return k
    return None


def _dataset_z_travel(mcap_path):
    k = _dataset_key(mcap_path)
    return Z_TRAVEL_M.get(k) if k else None


def _predetect_path(mcap_path):
    """Cache produced by predetect.py: {first_lidar_pub_ns, gt_xy_path_m, detections}."""
    k = _dataset_key(mcap_path)
    return os.path.join(HERE, "predetect", k + ".json") if k else None


# --- C2: floor flatness/level ----------------------------------------------
def c2_z_floor(t, z, ann):
    per, _errs = {}, []
    for ph in ann["phases"]:
        lvl = ph.get("level")
        h = ann["levels"].get(lvl) if lvl else None
        if h is None:
            continue
        m = (t >= ph["t0"] + TRIM_S) & (t <= ph["t1"] - TRIM_S)
        if m.sum() < 3:
            continue
        e = np.abs(z[m] - h)
        per.setdefault(lvl, []).extend(e.tolist())
    summary = {k: float(np.mean(v)) for k, v in per.items()}
    allerr = [e for v in per.values() for e in v]
    return {
        "z_floor_err_m": float(np.mean(allerr)) if allerr else None,
        "z_floor_by_level": summary,
    }


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
        per.append(
            {
                "phase": f"{fr}->{to}",
                "net_dz": float(net),
                "gap": float(gap),
                "err_m": float(err),
                "monotonic_frac": mono,
            }
        )
        errs.append(err)
    return {"z_ramp_err_m": float(np.mean(errs)) if errs else None, "z_ramp_by_transition": per}


# --- C1: AprilTag 3D spread -------------------------------------------------
def _detect_tags(mcap_path):
    """EXPENSIVE, trajectory-independent: read the video, detect 36h11, solvePnP ->
    list of (t_ns, tag_id, tvec_cam[3]) (tag centre in the camera-optical frame).
    Used by predetect.py to build the cache; also the no-cache fallback."""
    import cv2
    from mcap.reader import make_reader

    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT)),
        cv2.aruco.DetectorParameters(),
    )
    s = TAG_SIZE_M
    objp = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]])
    out = []
    with open(mcap_path, "rb") as f:
        for _, _ch, m in make_reader(f).iter_messages(topics=["rt/frontvideo"]):
            img = cv2.imdecode(
                np.frombuffer(_decode_jpeg_bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE
            )
            corners, ids, _ = det.detectMarkers(img)
            if ids is None:
                continue
            for cn, idv in zip(corners, ids.flatten(), strict=False):
                und = cv2.fisheye.undistortPoints(cn.reshape(-1, 1, 2).astype(np.float64), K, DIST)
                ok, _rvec, tvec = cv2.solvePnP(
                    objp, und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if ok:
                    out.append((int(m.publish_time), int(idv), [float(x) for x in tvec.ravel()]))
    return out


def c1_tag_spread(t_rel, pos, R, first_lidar_pub_ns, mcap_path, dets=None):
    """Project pre-detected tag poses (camera frame) to world via the INPUT trajectory,
    group same-id detections into per-visit tracks (>15 s gap), measure their spread.
    `dets` = [(t_ns, id, tvec_cam)]; loaded from the predetect cache when None (no mcap/cv2)."""
    if dets is None:
        cache = _predetect_path(mcap_path)
        if cache and os.path.exists(cache):
            dets = [(d["t_ns"], d["id"], d["tvec"]) for d in json.load(open(cache))["detections"]]
        else:
            dets = _detect_tags(mcap_path)
    traj_abs = first_lidar_pub_ns + (t_rel * 1e9)  # video on the same clock (≈; latency « drift)
    byid = {}  # id -> list of (t_ns, world_xyz)
    for t_ns, idv, tvec in dets:
        i = int(np.clip(np.searchsorted(traj_abs, t_ns), 0, len(traj_abs) - 1))
        p_base = R_OPT2LINK @ np.asarray(tvec) + T_LINK2BASE
        byid.setdefault(int(idv), []).append((t_ns, R[i] @ p_base + pos[i]))
    per, spreads = {}, []
    for idv, lst in byid.items():
        lst.sort()
        ts = np.array([d[0] for d in lst])
        W = np.array([d[1] for d in lst])
        groups = np.split(np.arange(len(lst)), np.where(np.diff(ts) / 1e9 > VISIT_GAP_S)[0] + 1)
        cents = np.array([W[g].mean(0) for g in groups])
        sp = (
            float(
                np.mean(
                    [
                        np.linalg.norm(cents[i] - cents[j])
                        for i, j in combinations(range(len(cents)), 2)
                    ]
                )
            )
            if len(cents) > 1
            else None
        )
        per[idv] = {"detections": len(lst), "visits": len(cents), "spread_m": sp}
        if sp is not None:
            spreads.append(sp)
    return {"tag_spread_m": float(np.mean(spreads)) if spreads else None, "tag_by_id": per}


# --- top-level --------------------------------------------------------------
def score_3d(t_rel, pos, R, first_lidar_pub_ns, mcap_path, ann, gt_xy_path, with_tags=True):
    z = _anchor_z(t_rel, pos[:, 2].copy(), ann)
    out = {}
    out.update(c2_z_floor(t_rel, z, ann))
    out.update(c3_z_ramp(t_rel, z, ann))
    if with_tags:
        try:
            out.update(c1_tag_spread(t_rel, pos, R, first_lidar_pub_ns, mcap_path))
        except Exception as e:  # cv2/mcap missing or no tags — keep z criteria usable
            out["tag_spread_m"] = None
            out["tag_error"] = repr(e)

    # --- two separate path-length scores: horizontal jitter vs vertical travel ---
    # xy: horizontal path length vs the gt horizontal path (over-travel = jitter).
    # z : total floor change actually traversed = Σ per-transition |net Δz| (bob-robust;
    #     NOT total-variation, which is dominated by body-bob noise) vs the measured total.
    lio_xy = _path_len(pos[:, :2])
    climbed = float(sum(abs(tr["net_dz"]) for tr in out.get("z_ramp_by_transition", [])))
    exp_z = _dataset_z_travel(mcap_path)
    xy_path_score = max(0.0, lio_xy / gt_xy_path - 1.0) if gt_xy_path else None
    z_path_score = abs(climbed / exp_z - 1.0) if exp_z else None
    out["xy_path_score"] = xy_path_score
    out["z_path_score"] = z_path_score
    out["path"] = {
        "lio_xy_m": lio_xy,
        "gt_xy_m": gt_xy_path,
        "climbed_z_m": climbed,
        "expected_z_m": exp_z,
    }

    # --- combined score (lower = better; None terms drop to 0) ---
    out["score"] = float(
        (out.get("tag_spread_m") or 0.0)
        + (out.get("z_floor_err_m") or 0.0)
        + (out.get("z_ramp_err_m") or 0.0)
        + (xy_path_score or 0.0)
        + (z_path_score or 0.0)
    )
    return out


if __name__ == "__main__":
    import argparse
    import json

    ap = argparse.ArgumentParser(description="3D eval (tag spread + z floor + z ramp)")
    ap.add_argument("--mcap", required=True, help="recording (.mcap) for tag frames + clock")
    ap.add_argument("--ann", required=True, help="annotations.json")
    ap.add_argument("--traj", help="mat_out.txt to score; omitted -> use robot_odom from --mcap")
    ap.add_argument("--no-tags", action="store_true")
    ap.add_argument(
        "--allow-fallback",
        action="store_true",
        help="allow the slow no-cache path (read mcap + detect). Default off: require a "
        "predetect cache; if absent, print empty JSON and exit (run predetect.py first).",
    )
    a = ap.parse_args()
    cache = _predetect_path(a.mcap)
    cdat = json.load(open(cache)) if (cache and os.path.exists(cache)) else None
    if cdat is None and not a.allow_fallback:
        print(json.dumps({}))  # no predetect cache and fallback disabled — nothing to score
        raise SystemExit(0)
    ann = json.load(open(a.ann))
    if a.traj:
        t, pos, R = load_mat_out(a.traj)
        if cdat:  # FAST path: constants from the predetect cache; no robot_odom mcap pass
            flp, gt_xy_path = cdat["first_lidar_pub_ns"], cdat["gt_xy_path_m"]
        else:  # un-preprocessed fallback
            _, rt_pos, _, flp = load_robot_odom(a.mcap)
            gt_xy_path = _path_len(rt_pos[:, :2])
    else:  # scoring robot_odom itself (baseline) — needs the odom pass
        t, pos, R, flp = load_robot_odom(a.mcap)
        gt_xy_path = _path_len(pos[:, :2])
    res = score_3d(t, pos, R, flp, a.mcap, ann, gt_xy_path, with_tags=not a.no_tags)
    print(json.dumps(res, indent=2))
