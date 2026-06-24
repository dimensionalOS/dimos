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

# Untyped analysis script: gtsam/open3d/cv2 lack type stubs.
# mypy: ignore-errors
"""AprilTag-loop-closed + ICP-refined ground-truth post-processing for a go2 recording.

Source of tags = the UNFILTERED tag stream (build it first with detect_tags.py). Each raw
detection carries its gate diagnostics, so gates are applied here post-hoc (no re-detection)
and are easy to relax. Factors: one robust (best-reproj) observation per keyframe x marker ->
denser, balanced loop closure than one-medoid-per-visit.

Two-stage solve: (1) GTSAM tag PGO -- anisotropic odometry between-factors (stiff roll/pitch + z
anchor gravity, loose yaw) + quality-weighted AprilTag landmark factors fix macro drift;
(2) ICP loop closures between spatially-close / temporally-distant lidar submaps anchor local
geometry. Writes <out>_odometry / <out>_lidar back into the recording db, optionally a .pc2.lcm
log of the corrected cloud, and opens a comparison rrd.

Usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/post_process.py [odom|lidar|both] --rec=PATH
       [--lidar=pointlio_lidar] [--odom=pointlio_odometry] [--tags=raw_april_tags]
       [--out=gt_pointlio] [--suffix=...] [--ignore-tags=17] [--no-icp] [--no-lcm] [--no-rrd]
"""

import json
from pathlib import Path
import sqlite3
import sys
import time

from gtsam import (
    BetweenFactorPose3,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    Point3,
    Pose3,
    PriorFactorPose3,
    Rot3,
    Symbol,
    Values,
    noiseModel,
)
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltags import (
    DEFAULT_MAX_ANGULAR_SPEED_DPS,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_LINEAR_SPEED_MPS,
    DEFAULT_MAX_REPROJ_PX,
    DEFAULT_MAX_VIEW_ANGLE_DEG,
    DEFAULT_MIN_SHARPNESS,
    DEFAULT_MIN_TAG_PX,
)

VISIT_GAP_S = 30.0
WHAT = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "both"


def arg(flag, default=""):
    return next((a.split("=", 1)[1] for a in sys.argv if a.startswith(flag + "=")), default)


REC_ARG = arg("--rec")
SUFFIX = arg("--suffix")
LIDAR_STREAM = arg("--lidar", "pointlio_lidar")  # input lidar stream (world-registered scans)
ODOM_STREAM = arg("--odom", "pointlio_odometry")  # input odometry stream (keyframe source)
RAW_STREAM = arg("--tags", "raw_april_tags")  # input unfiltered AprilTag stream
IGNORE_TAGS = {
    int(x) for x in arg("--ignore-tags").replace(",", " ").split()
}  # dynamic/moving tags
OUT_PREFIX = arg("--out", "gt_pointlio")  # output prefix -> <out>_odometry / <out>_lidar
WRITE_LCM = "--no-lcm" not in sys.argv  # also emit <out>_lidar.pc2.lcm of the corrected cloud
OPEN_RRD = "--no-rrd" not in sys.argv  # build + open a comparison rrd at the end
LCM_VOXEL = float(arg("--lcm-voxel", "0.05"))  # voxel size for the aggregated .pc2.lcm cloud
LCM_OUTLIER_NN = 20  # statistical outlier removal: neighbor count
LCM_OUTLIER_STD = 2.0  # ...and std-ratio threshold (lower = more aggressive)
LIDAR_FRAME = arg("--lidar-frame", "mid360_link")  # frame the raw lidar scans live in
WORLD_FRAME = arg("--world-frame", "world")  # frame to register scans into
USE_TF = "--no-tf" not in sys.argv  # world-register via recording tf (fallback: obs.pose)
TF_TOL = float(arg("--tf-tol", "0.5"))  # max seconds to the nearest recorded transform

# Per-glimpse gates (speed == -1 means "unknown" and always passes).
GATE = dict(
    min_sharpness=DEFAULT_MIN_SHARPNESS,
    max_reproj_px=DEFAULT_MAX_REPROJ_PX,
    min_tag_px=DEFAULT_MIN_TAG_PX,
    max_distance_m=DEFAULT_MAX_DISTANCE_M,
    max_view_angle_deg=DEFAULT_MAX_VIEW_ANGLE_DEG,
    max_lin_speed=DEFAULT_MAX_LINEAR_SPEED_MPS,
    max_ang_speed=DEFAULT_MAX_ANGULAR_SPEED_DPS,
)

if not REC_ARG:
    sys.exit(
        "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/post_process.py [odom|lidar|both] --rec=PATH "
        "[--lidar=...] [--odom=...] [--tags=...] [--out=...] [--suffix=...] "
        "[--no-icp] [--no-lcm] [--no-rrd]   (--rec is required: path to the recording dir)"
    )
REC = Path(REC_ARG).expanduser()
DB = REC / "mem2.db"
intr = json.loads((REC / "camera_intrinsics.json").read_text())
ext = np.array(intr["optical_in_base"], float)
Tbo = Pose3(Rot3.Quaternion(ext[6], ext[3], ext[4], ext[5]), Point3(ext[0], ext[1], ext[2]))
st = rdb.store(DB)
if RAW_STREAM not in st.list_streams():
    sys.exit(
        f"!! {RAW_STREAM} missing -- run detect_tags.py first to build the unfiltered tag stream."
    )
from dimos.memory2.db_tf import transform_matrix

_TF_AVAILABLE = USE_TF and st.tf.has_transforms()


def world_points(obs):
    """Nx3 world-registered points for a lidar observation.

    Primary: the recording's tf (``world <- LIDAR_FRAME``) at the scan time.
    Fallback: the observation's stored pose as the transform. Otherwise the
    scan is assumed already world-registered (legacy recordings).
    """
    pts = np.asarray(obs.data.points_f32())
    if not len(pts):
        return pts
    if _TF_AVAILABLE:
        tf = st.tf.get(WORLD_FRAME, LIDAR_FRAME, float(obs.ts), TF_TOL)
        if tf is not None:
            rot, trans = transform_matrix(tf)
            return pts @ rot.T + trans
    pose = getattr(obs, "pose", None)
    if isinstance(pose, (tuple, list)) and len(pose) >= 7:
        rot = Rot3.Quaternion(pose[6], pose[3], pose[4], pose[5]).matrix()
        return pts @ rot.T + np.array(pose[:3], float)
    return pts  # already world-registered


print(f"recording: {REC}", flush=True)
if _TF_AVAILABLE:
    print(f"world-registering {LIDAR_STREAM} via tf ({WORLD_FRAME} <- {LIDAR_FRAME})", flush=True)
print(
    f"streams: tags={RAW_STREAM} odom={ODOM_STREAM} lidar={LIDAR_STREAM} -> out={OUT_PREFIX}{SUFFIX}",
    flush=True,
)


def passes(t):
    return (
        t["sharpness"] >= GATE["min_sharpness"]
        and t["reproj_px"] <= GATE["max_reproj_px"]
        and t["tag_px"] >= GATE["min_tag_px"]
        and t["distance_m"] <= GATE["max_distance_m"]
        and t["view_angle_deg"] <= GATE["max_view_angle_deg"]
        and (t["lin_speed"] < 0 or t["lin_speed"] <= GATE["max_lin_speed"])
        and (t["ang_speed"] < 0 or t["ang_speed"] <= GATE["max_ang_speed"])
    )


# read raw detections (pose + diagnostics)
print("reading tag detections...", flush=True)
raw = []
for obs in st.stream(RAW_STREAM):
    ps = obs.data
    tg = obs.tags
    raw.append(
        dict(
            ts=float(obs.ts),
            marker_id=int(tg["marker_id"]),
            T_cam_tag=Pose3(
                Rot3.Quaternion(
                    ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z
                ),
                Point3(ps.x, ps.y, ps.z),
            ),
            reproj_px=float(tg["reproj_px"]),
            **{
                k: float(tg[k])
                for k in (
                    "sharpness",
                    "tag_px",
                    "distance_m",
                    "view_angle_deg",
                    "lin_speed",
                    "ang_speed",
                )
            },
        )
    )
gated = [t for t in raw if passes(t) and t["marker_id"] not in IGNORE_TAGS]

# keyframes from raw odometry
con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
fo = np.array(
    list(
        con.execute(
            "select ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
            f"from {ODOM_STREAM} order by ts"
        )
    ),
    float,
)
con.close()


def pr(r):
    return Rot3.Quaternion(r[7], r[4], r[5], r[6]), np.array(r[1:4])


ki = [0]
prev_rot, prev_pos = pr(fo[0])
for i in range(1, len(fo)):
    rot, pos = pr(fo[i])
    if (
        np.linalg.norm(pos - prev_pos) > 0.5
        or np.degrees(np.linalg.norm(Rot3.Logmap(prev_rot.inverse() * rot))) > 10
    ):
        ki.append(i)
        prev_rot, prev_pos = rot, pos
kfs = [pr(fo[i]) for i in ki]
kts = fo[ki, 0]
N = len(kfs)

# one factor per keyframe x marker: keep the best-reproj detection in each bucket
bucket = {}
for t in gated:
    kf = int(np.argmin(np.abs(kts - t["ts"])))
    key = (kf, t["marker_id"])
    if key not in bucket or t["reproj_px"] < bucket[key]["reproj_px"]:
        bucket[key] = t

# revisit report
raw_by, vis_by = {}, {}
for t in raw:
    raw_by.setdefault(t["marker_id"], 0)
    raw_by[t["marker_id"]] += 1
for (_kf, mid), t in bucket.items():
    vis_by.setdefault(mid, []).append(t["ts"])


def n_visits(times):
    times = sorted(times)
    visits = [[times[0]]]
    for tt in times[1:]:
        (visits[-1].append(tt) if tt - visits[-1][-1] <= VISIT_GAP_S else visits.append([tt]))
    return len(visits)


print(f"gates: {GATE}")
print(
    f"raw detections {len(raw)} -> {len(gated)} pass gates -> {len(bucket)} keyframe-tag factors\n"
)
print(f"{'tag':>4} | {'raw viewings':>12} | {'filtered revisits':>17}")
not_revisited = []
for mid in sorted(raw_by):
    nv = n_visits(vis_by[mid]) if mid in vis_by else 0
    flag = "" if nv >= 2 else "   <-- NOT REVISITED"
    print(f"{mid:>4} | {raw_by[mid]:>12} | {nv:>10} visit(s){flag}")
    if nv < 2:
        not_revisited.append(mid)
print(
    f"\ntags NOT revisited (no loop-closure constraint): {not_revisited if not_revisited else 'none'}\n"
)

# factor graph + solve
odom = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
grav0 = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-6, 1e-8, 1e-8, 1e-8]))


# quality weighting: planar-PnP pose error grows ~quadratically with range, and reproj_px is a
# direct misfit proxy. Inflate a glimpse's covariance by (dist/REF_D)^2 * (reproj/REF_R)^2 so a
# far/oblique/blurry tag pose contributes almost nothing while close, sharp ones dominate.
REF_D, REF_R = 0.4, 1.0


def tn(Rbt, distance_m=REF_D, reproj_px=REF_R):
    s = max((max(distance_m, 0.2) / REF_D) ** 2 * (max(reproj_px, 0.5) / REF_R) ** 2, 0.25)
    Rm = Rbt.matrix()
    c = np.zeros((6, 6))
    c[:3, :3] = Rm @ np.diag([0.04, 0.04, 0.0025]) @ Rm.T
    c[3:, 3:] = Rm @ np.diag([0.0025, 0.0025, 0.25]) @ Rm.T
    return noiseModel.Gaussian.Covariance(c * s)


print(f"building factor graph over {N} keyframes...", flush=True)
g = NonlinearFactorGraph()
v = Values()
for i in range(N):
    rot, pos = kfs[i]
    v.insert(i, Pose3(rot, Point3(pos)))
    if i == 0:
        g.add(PriorFactorPose3(0, Pose3(rot, Point3(pos)), grav0))
    else:
        rot_prev, pos_prev = kfs[i - 1]
        g.add(
            BetweenFactorPose3(
                i - 1,
                i,
                Pose3(
                    rot_prev.inverse() * rot,
                    Point3(rot_prev.inverse().rotate(Point3(pos - pos_prev))),
                ),
                odom,
            )
        )
seen = set()
for (kf, mid), t in sorted(bucket.items()):
    rot, pos = kfs[kf]
    kp = Pose3(rot, Point3(pos))
    T = Tbo.compose(t["T_cam_tag"])
    if mid not in seen:
        seen.add(mid)
        v.insert(Symbol("l", mid).key(), kp.compose(T))
    g.add(
        BetweenFactorPose3(
            kf, Symbol("l", mid).key(), T, tn(T.rotation(), t["distance_m"], t["reproj_px"])
        )
    )

print("solving stage 1 (tag PGO)...", flush=True)
lm_params = LevenbergMarquardtParams()
lm_params.setMaxIterations(200)
est = LevenbergMarquardtOptimizer(g, v, lm_params).optimize()
raw_kf = [Pose3(kfs[i][0], Point3(kfs[i][1])) for i in range(N)]

# STAGE 2: ICP loop-closure refinement (tags=macro, lidar ICP=local anchor)
# Tag PGO has pulled revisits roughly together; now ICP the lidar submaps of spatially-close,
# temporally-distant keyframe pairs to add precise 6-DOF relative constraints, then re-solve.
ICP = "--no-icp" not in sys.argv
if ICP:
    import open3d as o3d
    from scipy.spatial import cKDTree

    ICP_RADIUS_M = 4.0  # tag-corrected positions must be within this to be a revisit candidate
    ICP_MIN_DT_S = 25.0  # ...and at least this far apart in time (a real revisit, not adjacency)
    ICP_MAX_CORR_M = 0.6  # ICP correspondence distance
    ICP_VOXEL = 0.15
    ICP_FIT_MIN, ICP_RMSE_MAX = 0.45, 0.25
    SUBMAP_HALF_S = 1.0  # accumulate scans within +/- this of a keyframe time into its submap

    corr = [est.atPose3(i) for i in range(N)]
    cpos = np.array([np.asarray(p.translation()) for p in corr])

    # revisit candidate pairs
    tree = cKDTree(cpos)
    pairs = set()
    for i, j in tree.query_pairs(ICP_RADIUS_M):
        if abs(kts[i] - kts[j]) >= ICP_MIN_DT_S:
            pairs.add((min(i, j), max(i, j)))
    pairs = sorted(pairs, key=lambda p: np.linalg.norm(cpos[p[0]] - cpos[p[1]]))
    involved = {k for p in pairs for k in p}
    print(
        f"ICP stage: {len(pairs)} revisit candidate pairs over {len(involved)} keyframes",
        flush=True,
    )

    # build per-(involved)-keyframe body-frame submaps from the input lidar (odom-registered)
    print("ICP stage: reading lidar submaps...", flush=True)
    submap = {k: [] for k in involved}
    scan_n = 0
    t_sub = time.time()
    for obs in st.stream(LIDAR_STREAM):
        scan_n += 1
        if scan_n % 20000 == 0:
            print(f"  read {scan_n} scans, {time.time() - t_sub:.0f}s", flush=True)
        ts = float(obs.ts)
        k = int(np.argmin(np.abs(kts - ts)))
        if k not in submap or abs(kts[k] - ts) > SUBMAP_HALF_S:
            continue
        rot_k, pos_k = kfs[k]
        world = world_points(obs)
        submap[k].append((world - pos_k) @ rot_k.matrix())  # world -> kf-k body frame
    pcd = {}
    for k, chunks in submap.items():
        if not chunks:
            continue
        p = o3d.geometry.PointCloud()
        p.points = o3d.utility.Vector3dVector(np.concatenate(chunks, 0).astype(np.float64))
        p = p.voxel_down_sample(ICP_VOXEL)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        pcd[k] = p
    print(f"ICP stage: built {len(pcd)} submaps, registering {len(pairs)} pairs...", flush=True)

    icp_noise = noiseModel.Robust.Create(
        noiseModel.mEstimator.Huber.Create(1.345),
        noiseModel.Diagonal.Variances(np.array([4e-4, 4e-4, 4e-4, 2.5e-3, 2.5e-3, 2.5e-3])),
    )
    added = 0
    t_icp = time.time()
    for pair_n, (i, j) in enumerate(pairs):
        if pair_n and pair_n % 5000 == 0:
            print(
                f"  registered {pair_n}/{len(pairs)} pairs, {added} accepted, {time.time() - t_icp:.0f}s",
                flush=True,
            )
        if i not in pcd or j not in pcd:
            continue
        init = (corr[i].inverse() * corr[j]).matrix()  # i<-j initial guess from tag correction
        res = o3d.pipelines.registration.registration_icp(
            pcd[j],
            pcd[i],
            ICP_MAX_CORR_M,
            init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        if res.fitness >= ICP_FIT_MIN and res.inlier_rmse <= ICP_RMSE_MAX:
            T = res.transformation
            g.add(BetweenFactorPose3(i, j, Pose3(Rot3(T[:3, :3]), Point3(T[:3, 3])), icp_noise))
            added += 1
    print(
        f"ICP stage: accepted {added}/{len(pairs)} loop closures "
        f"(fit>={ICP_FIT_MIN}, rmse<={ICP_RMSE_MAX}m)",
        flush=True,
    )
    if added:
        # est already holds every key (keyframe + landmark poses); warm-start from it
        print("solving stage 2 (tag PGO + ICP closures)...", flush=True)
        est = LevenbergMarquardtOptimizer(g, est, lm_params).optimize()

C = [est.atPose3(i).compose(raw_kf[i].inverse()) for i in range(N)]
shift = max(float(np.linalg.norm(np.asarray(C[i].translation()))) for i in range(N))
print(
    f"PGO: N={N} keyframes, {len(bucket)} tag factors over {len(seen)} markers, "
    f"max correction shift {shift:.1f} m",
    flush=True,
)


def C_at(ts):
    if ts <= kts[0]:
        return C[0]
    if ts >= kts[-1]:
        return C[-1]
    j = int(np.searchsorted(kts, ts))
    a, b = j - 1, j
    al = (ts - kts[a]) / (kts[b] - kts[a])
    return C[a].compose(Pose3.Expmap(al * Pose3.Logmap(C[a].between(C[b]))))


def pose_tuple(P):
    t = P.translation()
    q = P.rotation().toQuaternion()
    return (t[0], t[1], t[2], q.x(), q.y(), q.z(), q.w())


if WHAT in ("odom", "both"):
    name = f"{OUT_PREFIX}_odometry{SUFFIX}"
    if name in st.list_streams():
        st.delete_stream(name)
    out = st.stream(name, Odometry)
    print(f"writing {name} ({len(fo)} poses)...", flush=True)
    n = 0
    t0 = time.time()
    for r in fo:
        ts = float(r[0])
        P = C_at(ts).compose(
            Pose3(Rot3.Quaternion(r[7], r[4], r[5], r[6]), Point3(r[1], r[2], r[3]))
        )
        x, y, zz, qx, qy, qz, qw = pose_tuple(P)
        out.append(
            Odometry(
                ts=ts,
                frame_id="odom",
                child_frame_id="base_link",
                pose=Pose(x, y, zz, qx, qy, qz, qw),
            ),
            ts=ts,
            pose=(x, y, zz, qx, qy, qz, qw),
        )
        n += 1
        if n % 20000 == 0:
            print(f"  {n}/{len(fo)} poses, {time.time() - t0:.0f}s", flush=True)
    print(f"wrote {name}: {len(fo)} poses in {time.time() - t0:.0f}s", flush=True)

if WHAT in ("lidar", "both"):
    name = f"{OUT_PREFIX}_lidar{SUFFIX}"
    fo_ts = fo[:, 0]

    def base_pose(ts):
        j = int(np.searchsorted(fo_ts, ts))
        j = min(max(j, 0), len(fo) - 1)
        if j > 0 and abs(fo_ts[j - 1] - ts) < abs(fo_ts[j] - ts):
            j -= 1
        r = fo[j]
        return Pose3(Rot3.Quaternion(r[7], r[4], r[5], r[6]), Point3(r[1], r[2], r[3]))

    if name in st.list_streams():
        st.delete_stream(name)
    out = st.stream(name, PointCloud2)

    # The db stream stays per-scan, but the .pc2.lcm is ONE aggregated cloud (voxel-downsampled +
    # statistical-outlier-removed), not 5184 per-scan events. Intensity rides through open3d's
    # voxel averaging via the color channel. Chunks are collapsed every CHUNK scans to bound memory.
    if WRITE_LCM:
        import open3d as o3d

    CHUNK = 1000
    agg_xyz, agg_i = [], []  # incrementally voxel-downsampled chunks
    buf_xyz, buf_i = [], []
    have_inten = False

    def collapse(xyz_list, i_list, voxel):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.concatenate(xyz_list).astype(np.float64))
        carry = bool(i_list)
        if carry:
            inten_col = np.concatenate(i_list).astype(np.float64)[:, None]
            pc.colors = o3d.utility.Vector3dVector(np.repeat(inten_col, 3, axis=1))
        pc = pc.voxel_down_sample(voxel)
        dx = np.asarray(pc.points, np.float32)
        di = np.asarray(pc.colors, np.float32)[:, 0] if carry else None
        return dx, di

    print(f"writing {name} (corrected lidar)...", flush=True)
    n = 0
    t0 = time.time()
    for obs in st.stream(LIDAR_STREAM):
        ts = float(obs.ts)
        Cts = C_at(ts)
        Rm = Cts.rotation().matrix()
        tv = np.asarray(Cts.translation())
        xyz = world_points(obs)
        inten = obs.data.intensities_f32()
        xyz2 = (xyz @ Rm.T + tv).astype(np.float32)
        m = PointCloud2.from_numpy(
            xyz2, frame_id="odom", intensities=(np.asarray(inten) if inten is not None else None)
        )
        m.ts = ts  # stamp the cloud (lcm_encode needs a non-None ts)
        out.append(m, ts=ts, pose=pose_tuple(Cts.compose(base_pose(ts))))
        if WRITE_LCM:
            buf_xyz.append(xyz2)
            if inten is not None:
                have_inten = True
                buf_i.append(np.asarray(inten, np.float32))
            if len(buf_xyz) >= CHUNK:
                dx, di = collapse(buf_xyz, buf_i if have_inten else [], LCM_VOXEL)
                agg_xyz.append(dx)
                if di is not None:
                    agg_i.append(di)
                buf_xyz, buf_i = [], []
        n += 1
        if n % 2000 == 0:
            print(f"  {n} scans, {time.time() - t0:.0f}s", flush=True)
    print(f"wrote {name}: {n} scans in {time.time() - t0:.0f}s", flush=True)

    if WRITE_LCM:
        if buf_xyz:  # flush remainder
            dx, di = collapse(buf_xyz, buf_i if have_inten else [], LCM_VOXEL)
            agg_xyz.append(dx)
            if di is not None:
                agg_i.append(di)
        # final unified voxel pass over the per-chunk results, then statistical outlier removal
        merged_i = agg_i if have_inten else []
        dx, di = collapse(agg_xyz, merged_i, LCM_VOXEL)
        print(
            f"aggregating .pc2.lcm: {len(dx):,} pts after voxel, removing outliers...", flush=True
        )
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(dx.astype(np.float64))
        if di is not None:
            pc.colors = o3d.utility.Vector3dVector(
                np.repeat(di.astype(np.float64)[:, None], 3, axis=1)
            )
        pc, _keep = pc.remove_statistical_outlier(LCM_OUTLIER_NN, LCM_OUTLIER_STD)
        merged_xyz = np.asarray(pc.points, np.float32)
        merged_inten = np.asarray(pc.colors, np.float32)[:, 0] if di is not None else None
        merged = PointCloud2.from_numpy(merged_xyz, frame_id="odom", intensities=merged_inten)
        merged.ts = float(fo_ts[0])
        lcm_path = REC / f"{name}.pc2.lcm"
        lcm_path.write_bytes(merged.lcm_encode())
        print(
            f"wrote {lcm_path}: 1 aggregated cloud, {len(merged_xyz):,} pts "
            f"(voxel {LCM_VOXEL} m, outlier nn={LCM_OUTLIER_NN}/std={LCM_OUTLIER_STD})",
            flush=True,
        )

# build + open the comparison rrd
if OPEN_RRD and WHAT in ("lidar", "both"):
    import subprocess

    from dimos.navigation.jnav.components.loop_closure.gsc_pgo import make_rrd

    print("building comparison rrd...", flush=True)
    rrd_path = make_rrd.build(
        REC, lidar_stream=LIDAR_STREAM, odom_stream=ODOM_STREAM, tag_stream=RAW_STREAM
    )
    rerun_bin = Path(sys.executable).parent / "rerun"
    if rerun_bin.exists():
        subprocess.Popen([str(rerun_bin), str(rrd_path)])
        print(f"opened {rrd_path}", flush=True)
    else:
        print(f"rerun binary not found at {rerun_bin}; open manually: rerun {rrd_path}", flush=True)
