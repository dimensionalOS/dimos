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

"""Closed-loop hand-eye calibration against a ChArUco board, with its own verdict.

``solve_hand_eye`` drives an arm through poses drawn about a home pose, detects the board in
every frame, and solves for the camera pose: on the flange for a wrist camera (eye in hand) or
in the base for a fixed camera watching a board held by the arm (eye to hand). OpenCV's Park
method gives the starting guess and a joint Levenberg-Marquardt refinement of the camera pose
and the board pose against the pixel residuals gives the answer.

The loop is closed on the images: the draw narrows after a frame with no board and widens back
after a hit, and the centre of the draw walks the board to ``PROBE_M`` in front of the camera,
the distance a grasp is felt at, because a solve is accurate at the distance the board sat.

The verdict re-solves on resampled subsets of the poses and reads the scatter of the point
``PROBE_M`` in front of the camera, checks that the board lands in one place from every pose,
and refuses a solution far from the stated nominal mount. ``passed`` is False unless all hold.

The robot and camera are given as the two small protocols below, so the same solver runs on a
deployed stack and on a mocked one in tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, NamedTuple, Protocol

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FloatArray = NDArray[np.float64]

CameraFrameKind = Literal["flange", "base"]

# Where the answer is read off: a grasp is felt about this far in front of the camera, so it is
# also the distance the board is walked to.
PROBE_M = 0.4

# Pose draw: how far each joint may move from the centre in one pose, radians, scaled by the
# amplitude; the widest amplitude an arrangement affords; and how the amplitude reacts to a frame
# without a board (shrink) and with one (recover).
STEP_RANGE = np.array([0.25, 0.15, 0.15, 0.5, 0.35, 0.8])
AMP_CEIL = {
    False: 1.0,
    True: 0.5,
}  # keyed by eye_to_hand: a board on the arm leaves the frame sooner
AMP_SHRINK = 0.6
AMP_RECOVER = 1.6
AMP_MIN = 0.2

# Standoff servo: the centre of the draw walks until the views average to PROBE_M.
DEPTH_TOL_M = 0.03
STANDOFF_MIN_M = 0.33  # the nearest an approach reaches; a setpoint below it only loses views
DEPTH_FIT_MIN = 7  # views needed before the fitted joint-to-view map can steer
DEPTH_FIT_R2 = 0.8
DEPTH_STEP_M = 0.20  # trust region in metres of standoff per step
DEPTH_STEP_MIN_M = 0.02
DEPTH_RIDGE = 1e-3
CENTRE_SPAN_RAD = 0.8  # the centre never wanders further than this from home, per joint
STEP_MAX_RAD = 0.25
MISS_RETREAT = 2  # consecutive blind or refused poses that call off an approach
OBS_W = np.array([50.0, 50.0, 50.0, 50.0, 50.0, 10.0])  # 1/tolerance per observable
WALK_DAMP = 1e-3

# Verdict.
N_RESAMPLE = 12
RESAMPLE_FRAC = 0.7
PASS_MM = 2.0  # estimated probe-point error must be under this
CATASTROPHE_MM = 4.0  # and no resample may look like a disaster
# The board must land in one place from every pose. A real arm's absolute accuracy, a few
# millimetres over its reach, lands here as well as the camera does, so the gate is wider than
# the solver alone would need (a perfect-kinematics twin holds 0.1 mm).
MISCLOSE_MM = 4.0
MIN_POSES = 4
NOMINAL_TOL_MULT = 2.0  # a solve further than this many tolerances from the nominal is refused

# Detection and refinement.
CORE_MIN_MARKERS = 2  # OpenCV's default: corners bracketed by two decoded markers
WIDE_MIN_MARKERS = 1  # the sighting test only, never the solve
MIN_CORNERS = 8
REJECT_MULT = 4.0
REJECT_FLOOR_PX = 0.5
HUBER_PX = 1.0
MAX_ITERS = 50
FD_STEP = 1e-7
PROBE_PT = np.array([0.0, 0.0, PROBE_M])
TILT_GRID = np.arange(0.0, 40.0, 0.5)
LEVER_TOL_M = 0.06
N_EFF_MIN = 14.0
SEED = 0


@dataclass(frozen=True)
class BoardSpec:
    """A ChArUco board: ``squares_x`` by ``squares_y`` squares of ``square_m`` with AprilTag
    markers of ``marker_m``, ids counting up from ``ids_from``."""

    squares_x: int
    squares_y: int
    square_m: float
    marker_m: float
    ids_from: int = 0
    dictionary: str = "DICT_APRILTAG_36h11"

    def ids(self) -> NDArray[np.int32]:
        n = self.squares_x * self.squares_y // 2
        return np.arange(self.ids_from, self.ids_from + n, dtype=np.int32)


@dataclass(frozen=True)
class CameraNominal:
    """Where the camera is believed to be, as a 4x4 in ``frame``, with the tolerance of that
    belief. The solver uses it only for the gross-blunder check of its verdict."""

    frame: CameraFrameKind
    T: FloatArray
    tol_mm: float
    tol_deg: float


@dataclass(frozen=True)
class Frame:
    """One captured colour frame with the camera's intrinsics."""

    image: NDArray[np.uint8]  # HxWx3 RGB
    K: FloatArray
    dist: FloatArray
    ts: float


class CalibrationRobot(Protocol):
    def joint_limits(self) -> list[tuple[float, float]]: ...

    def get_joints(self) -> list[float]: ...

    def get_flange_pose(self) -> FloatArray:
        """Base -> flange, 4x4, at the moment of the call."""
        ...

    def move_to_joints(self, q: list[float]) -> bool:
        """Blocking. False when the move was refused; the arm then stayed where it was."""
        ...

    def clock(self) -> float:
        """Seconds since the run started."""
        ...


class CalibrationCamera(Protocol):
    def capture(self) -> Frame:
        """A frame taken after the last move completed."""
        ...


@dataclass
class HandEyeResult:
    """The camera pose in ``frame`` and the solver's own verdict on it."""

    frame: CameraFrameKind
    T: FloatArray
    passed: bool
    poses_tried: int
    poses_moved: int
    confidence: dict[str, float] = field(default_factory=dict)
    reason: str = ""


class _Obs(NamedTuple):
    """One pose's evidence: the PnP pose for a starting guess, the corners for refining."""

    T_cam_board: FloatArray
    obj: FloatArray  # Nx3 board-frame corners
    img: FloatArray  # Nx2 detected image corners
    resid_px: float


def rt_to_T(Rm: FloatArray, t: FloatArray) -> FloatArray:
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(t, dtype=float).ravel()
    return T


def T_inv(T: FloatArray) -> FloatArray:
    Rm, t = T[:3, :3], T[:3, 3]
    return rt_to_T(Rm.T, -Rm.T @ t)


def charuco_board(spec: BoardSpec) -> cv2.aruco.CharucoBoard:
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y), spec.square_m, spec.marker_m, d, ids=spec.ids()
    )


def _detectors(
    board: cv2.aruco.CharucoBoard,
) -> tuple[cv2.aruco.CharucoDetector, cv2.aruco.CharucoDetector]:
    """The detector the solve sees, and the one that only says whether the board is in shot.

    A ChArUco corner on the board's own boundary has one decoded neighbour and localises an
    order of magnitude worse than the core corners, so it never enters the solve; but a
    half-visible board shows exactly those corners, so they are the right evidence for the
    amplitude feedback, which must not narrow the draw for a board that was still in shot.
    """
    pc, pw = cv2.aruco.CharucoParameters(), cv2.aruco.CharucoParameters()
    pc.minMarkers, pw.minMarkers = CORE_MIN_MARKERS, WIDE_MIN_MARKERS
    return cv2.aruco.CharucoDetector(board, pc), cv2.aruco.CharucoDetector(board, pw)


def detect_board(
    board: cv2.aruco.CharucoBoard,
    dets: tuple[cv2.aruco.CharucoDetector, cv2.aruco.CharucoDetector],
    frame: Frame,
    wide_seen: bool,
) -> tuple[bool, _Obs | None]:
    """(seen, obs): whether the board was in shot at all, and the view to solve on, if any."""
    det_core, det_wide = dets
    gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
    corners, ids, _, _ = det_core.detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        if not wide_seen:
            return False, None
        _, wide_ids, _, _ = det_wide.detectBoard(gray)
        return (wide_ids is not None and len(wide_ids) >= MIN_CORNERS), None
    # the stubs declare a sequence of corner arrays; the binding takes the matrix detectBoard returns
    obj, img = board.matchImagePoints(corners, ids)  # type: ignore[call-overload]
    ok, rvec, tvec = cv2.solvePnP(obj, img, frame.K, frame.dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return True, None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, frame.K, frame.dist)
    diff = proj.reshape(-1, 2) - img.reshape(-1, 2)
    resid = float(np.sqrt(np.mean(np.sum(diff**2, axis=1))))
    Rm, _ = cv2.Rodrigues(rvec)
    T = rt_to_T(np.asarray(Rm, dtype=float), np.asarray(tvec, dtype=float))
    return True, _Obs(T, obj.reshape(-1, 3).astype(float), img.reshape(-1, 2).astype(float), resid)


def _observe(T_cam_board: FloatArray, R0: FloatArray) -> FloatArray:
    """What the camera sees of the board as six numbers: attitude (against the first view, so
    it never wraps), bearing in the frame, and standoff. Only the standoff is to be changed."""
    r, _ = cv2.Rodrigues(T_cam_board[:3, :3] @ R0.T)
    t = T_cam_board[:3, 3]
    z = t[2] if abs(t[2]) > 1e-6 else 1e-6
    return np.array([r[0, 0], r[1, 0], r[2, 0], t[0] / z, t[1] / z, float(np.linalg.norm(t))])


def _standoff_target(standoffs: FloatArray, remaining: int) -> float | None:
    """The distance to ask the next views for so that the mean standoff over the whole run lands
    on PROBE_M: the views already banked far away are paid back by the ones still to come.
    One-sided and floored, so an arrangement already inside PROBE_M is left alone."""
    n = len(standoffs)
    if n == 0 or remaining < 1:
        return None
    deficit = float(np.mean(standoffs)) - PROBE_M
    if deficit < DEPTH_TOL_M:
        return None
    return max(PROBE_M - n * deficit / remaining, STANDOFF_MIN_M)


def _view_jacobian(Q: FloatArray, U: FloatArray) -> tuple[FloatArray, FloatArray]:
    """The local map from joints to the view, fitted on the views already taken, and the
    fraction of each observable's variation it explains."""
    Qc, Uc = Q - Q.mean(axis=0), U - U.mean(axis=0)
    s = np.where(Qc.std(axis=0) < 1e-9, 1.0, Qc.std(axis=0))
    Z = Qc / s
    G = np.linalg.solve(Z.T @ Z + DEPTH_RIDGE * len(Q) * np.eye(6), Z.T @ Uc)
    var = np.sum(Uc**2, axis=0)
    sse = np.sum((Uc - Z @ G) ** 2, axis=0)
    r2 = np.where(var > 1e-12, 1.0 - sse / np.where(var > 1e-12, var, 1.0), 0.0)
    return (G / s[:, None]).T, r2


def _walk_centre(
    centre: FloatArray,
    Q: FloatArray,
    U: FloatArray,
    home: FloatArray,
    bounds: tuple[FloatArray, FloatArray],
    step_cap: float,
    target_m: float,
) -> FloatArray:
    """Move the centre of the draw toward a standoff of target_m, holding the board's attitude
    and place in the frame, so the approach keeps its views. Clipped to a trust region."""
    J, r2 = _view_jacobian(Q, U)
    here = U[:, 5].mean() + float(J[5] @ (centre - Q.mean(axis=0)))
    want = target_m - here
    if r2[5] < DEPTH_FIT_R2 or abs(want) < DEPTH_TOL_M:
        return centre
    ask = np.zeros(6)
    ask[5] = np.clip(want, -step_cap, step_cap)
    w2 = OBS_W**2
    H = J.T @ (w2[:, None] * J)
    scale = float(np.mean(np.diag(H)))
    if not scale > 0.0:
        return centre
    step = np.linalg.solve(H + WALK_DAMP * scale * np.eye(6), J.T @ (w2 * ask))
    big = np.max(np.abs(step)) / STEP_MAX_RAD
    if big > 1.0:
        step /= big
    lo, hi = bounds
    out: FloatArray = np.clip(
        centre + step,
        np.maximum(lo, home - CENTRE_SPAN_RAD),
        np.minimum(hi, home + CENTRE_SPAN_RAD),
    )
    return out


def _retreat(
    centre: FloatArray, anchor: FloatArray, step_cap: float, misses: int
) -> tuple[FloatArray, FloatArray, float]:
    """After MISS_RETREAT blind or refused poses, go back to the last centre a view came from and
    halve the trust region, so an approach converges instead of grinding against a limit."""
    if misses < MISS_RETREAT:
        return centre, anchor, step_cap
    return anchor.copy(), anchor, step_cap * 0.5


def _handeye_park(A: list[FloatArray], B: list[FloatArray]) -> FloatArray:
    R_x, t_x = cv2.calibrateHandEye(
        [a[:3, :3] for a in A],
        [a[:3, 3] for a in A],
        [b[:3, :3] for b in B],
        [b[:3, 3] for b in B],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    return rt_to_T(np.asarray(R_x, dtype=float), np.asarray(t_x, dtype=float))


def _pack(T: FloatArray) -> FloatArray:
    r, _ = cv2.Rodrigues(T[:3, :3])
    out: FloatArray = np.concatenate([r.ravel(), T[:3, 3]])
    return out


def _unpack(p: FloatArray) -> FloatArray:
    Rm, _ = cv2.Rodrigues(np.asarray(p[:3], dtype=float))
    return rt_to_T(np.asarray(Rm, dtype=float), p[3:6])


def _mean_T(Ts: list[FloatArray]) -> FloatArray:
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(np.sum([T[:3, :3] for T in Ts], axis=0))
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        Rm = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    return rt_to_T(Rm, t)


def _target_pose(A: list[FloatArray], obs: list[_Obs], X: FloatArray) -> list[FloatArray]:
    """Z, the board pose, as each pose sees it: Z_i = A_i X B_i. With the right X every pose
    agrees, so the spread measures how far the chain is from closing on itself."""
    return [a @ X @ o.T_cam_board for a, o in zip(A, obs, strict=True)]


def _project(
    T: FloatArray, pts: FloatArray, pose_of: NDArray[np.intp], K: FloatArray, dist: FloatArray
) -> FloatArray:
    """Pinhole plus Brown-Conrady for every pose's corners in one pass; matches cv2.projectPoints."""
    R, t = T[:, :3, :3], T[:, :3, 3]
    q = np.einsum("nij,nj->ni", R[pose_of], pts) + t[pose_of]
    z = np.where(np.abs(q[:, 2]) < 1e-9, 1e-9, q[:, 2])
    x, y = q[:, 0] / z, q[:, 1] / z
    k1, k2, p1, p2, k3 = dist
    r2 = x * x + y * y
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    out: FloatArray = np.stack([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]], axis=1)
    return out


def _huber_w(res: FloatArray, c: float) -> FloatArray:
    e = np.linalg.norm(res, axis=1)
    out: FloatArray = np.where(e > c, c / np.maximum(e, 1e-12), 1.0)
    return out


def _view_centroids(obs: list[_Obs]) -> FloatArray:
    """Where each view's board sat in the camera frame: the centroid of its own corners."""
    rows = [o.T_cam_board[:3, :3] @ o.obj.mean(axis=0) + o.T_cam_board[:3, 3] for o in obs]
    return np.asarray(rows, dtype=float)


def _tilt_w(lam: FloatArray, d: FloatArray) -> FloatArray:
    s = d @ lam
    e = np.exp(s - s.max())
    return np.asarray(e / e.sum(), dtype=float)


def _lever_weights(obs: list[_Obs]) -> FloatArray | None:
    """Per-view weights that move the refinement's pivot toward the probe point.

    The weak direction of a reprojection solve is a rotation of the camera about the mean board
    position, so the error at the probe point is that rotation times the lever from there to the
    probe point. Reweighting shortens the lever at the price of effective views; the tilt is
    chosen by searching lever / sqrt(n_eff) on the views in hand, and None (equal weights) is
    returned whenever the trade does not pay.
    """
    d = _view_centroids(obs) - PROBE_PT
    lever = float(np.linalg.norm(d.mean(axis=0)))
    if len(d) < MIN_POSES or lever < LEVER_TOL_M:
        return None
    u = -d.mean(axis=0) / lever
    best, best_w = 1.0, None
    for a in TILT_GRID[1:]:
        w = _tilt_w(a * u, d)
        n_eff = 1.0 / float(np.sum(w**2))
        if n_eff < N_EFF_MIN:
            break
        obj = (np.linalg.norm(w @ d) / lever) * np.sqrt(len(d) / n_eff)
        if obj < best:
            best, best_w = obj, w
    return None if best_w is None else best_w * len(d)


def _wcost(res: FloatArray, w: FloatArray) -> float:
    return float(np.sum(w * np.sum(res**2, axis=1)))


def _refine(
    Ainv: FloatArray,
    obs: list[_Obs],
    X0: FloatArray,
    Z0: FloatArray,
    K: FloatArray,
    dist: FloatArray,
    view_w: FloatArray | None,
) -> tuple[FloatArray, FloatArray, float]:
    """Levenberg-Marquardt on the 12 parameters of X (camera) and Z (board), Huber-weighted on
    pixel residuals. T_cam_board = X^-1 A^-1 Z in both arrangements, with A already inverted by
    the caller for eye to hand. X and Z trade off against each other, so they are solved together.
    """
    pts: FloatArray = np.concatenate([o.obj for o in obs])
    img: FloatArray = np.concatenate([o.img for o in obs])
    n_per = [len(o.obj) for o in obs]
    pose_of = np.repeat(np.arange(len(obs)), n_per)
    vw = np.ones(len(pts)) if view_w is None else np.repeat(view_w, n_per)

    def residual(p: FloatArray) -> FloatArray:
        T = T_inv(_unpack(p[:6])) @ Ainv @ _unpack(p[6:])
        return _project(T, pts, pose_of, K, dist) - img

    p = np.concatenate([_pack(X0), _pack(Z0)])
    res = residual(p)
    w = _huber_w(res, HUBER_PX) * vw
    lam = 1e-3
    step = np.zeros(12)
    for _ in range(MAX_ITERS):
        sw = np.sqrt(np.repeat(w, 2))
        J = np.empty((res.size, 12))
        for j in range(12):
            q = p.copy()
            q[j] += FD_STEP
            J[:, j] = ((residual(q) - res).ravel() / FD_STEP) * sw
        H, g = J.T @ J, J.T @ (res.ravel() * sw)
        diag = np.maximum(np.diag(H), 1e-12)
        cost = _wcost(res, w)
        improved = False
        for _ in range(8):
            try:
                step = np.linalg.solve(H + lam * np.diag(diag), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            res_try = residual(p + step)
            if _wcost(res_try, w) < cost:
                p, res = p + step, res_try
                w = _huber_w(res, HUBER_PX) * vw
                lam = max(lam / 3.0, 1e-10)
                improved = True
                break
            lam *= 10.0
        if not improved or np.linalg.norm(step) < 1e-10:
            break
    rms_px = float(np.sqrt(np.mean(np.sum(res**2, axis=1))))
    return _unpack(p[:6]), _unpack(p[6:]), rms_px


def _reject_bad_views(A: list[FloatArray], obs: list[_Obs]) -> tuple[list[FloatArray], list[_Obs]]:
    """Drop whole views whose PnP residual stands out: a blurred frame the Huber loss would outvote
    in the solve still corrupts the misclosure check, which averages over poses."""
    r = np.array([o.resid_px for o in obs])
    keep = r <= max(REJECT_MULT * float(np.median(r)), REJECT_FLOOR_PX)
    if keep.sum() < max(MIN_POSES, 3):
        return A, obs
    return (
        [a for a, k in zip(A, keep, strict=True) if k],
        [o for o, k in zip(obs, keep, strict=True) if k],
    )


def _solve_pair(
    A: list[FloatArray],
    obs: list[_Obs],
    K: FloatArray,
    dist: FloatArray,
    X0: FloatArray | None = None,
    Z0: FloatArray | None = None,
) -> tuple[FloatArray, FloatArray, float]:
    if X0 is None:
        X0 = _handeye_park(A, [o.T_cam_board for o in obs])
    if Z0 is None:
        Z0 = _mean_T(_target_pose(A, obs, X0))
    Ainv = np.array([T_inv(a) for a in A])
    return _refine(Ainv, obs, X0, Z0, K, dist, _lever_weights(obs))


def _probe_error_mm(
    A: list[FloatArray],
    obs: list[_Obs],
    K: FloatArray,
    dist: FloatArray,
    X: FloatArray,
    Z: FloatArray,
) -> tuple[float, float]:
    """(typical, worst) scatter in mm of the point PROBE_M in front of the camera when the whole
    estimator is re-run on random subsets of the poses, each started from the full solution."""
    n = len(A)
    if n < MIN_POSES:
        return float("inf"), float("inf")
    rng = np.random.default_rng(SEED + 1)
    k = max(3, round(RESAMPLE_FRAC * n))
    d = np.array([0.0, 0.0, PROBE_M, 1.0])
    p0 = (X @ d)[:3]
    offs = []
    for _ in range(N_RESAMPLE):
        idx = rng.choice(n, size=k, replace=False)
        Xs, _, _ = _solve_pair([A[i] for i in idx], [obs[i] for i in idx], K, dist, X0=X, Z0=Z)
        offs.append(np.linalg.norm((Xs @ d)[:3] - p0))
    # subsets hold RESAMPLE_FRAC of the data, so their scatter overstates the full solve's
    scale = np.sqrt(RESAMPLE_FRAC) * 1000
    return float(np.mean(offs) * scale), float(np.max(offs) * scale)


def solve_hand_eye(
    robot: CalibrationRobot,
    camera: CalibrationCamera,
    board: BoardSpec,
    nominal: CameraNominal,
    home_joints: list[float],
    max_poses: int,
    max_seconds: float,
    seed: int = SEED,
) -> HandEyeResult:
    """Run the calibration. Returns the camera pose in ``nominal.frame`` and the verdict.

    Eye to hand (``nominal.frame == "base"``) means the board rides on the arm and the camera is
    fixed; the arm's role is the same, only the chain is inverted.
    """
    eye_to_hand = nominal.frame == "base"
    cb = charuco_board(board)
    dets = _detectors(cb)
    A: list[FloatArray] = []
    obs: list[_Obs] = []
    K: FloatArray | None = None
    dist: FloatArray | None = None
    rng = np.random.default_rng(seed)
    home = np.asarray(home_joints, dtype=float)
    ceil = AMP_CEIL[eye_to_hand]
    amp = ceil
    # the draw is kept a full swing clear of the joint limits so it stays symmetric about its centre
    lim = np.asarray(robot.joint_limits(), dtype=float)
    bounds = (lim[:, 0] + ceil * STEP_RANGE, lim[:, 1] - ceil * STEP_RANGE)
    centre: FloatArray = np.clip(home.copy(), *bounds)
    anchor = centre.copy()
    step_cap = DEPTH_STEP_M
    Q: list[FloatArray] = []
    U: list[FloatArray] = []
    misses = 0
    R0: FloatArray | None = None
    tried = moved = unsolved = 0
    while tried < max_poses and robot.clock() <= max_seconds - 5:
        q = centre + amp * rng.uniform(-1.0, 1.0, 6) * STEP_RANGE
        tried += 1
        if not robot.move_to_joints([float(v) for v in q]):
            amp = max(amp * AMP_SHRINK, AMP_MIN)
            misses += 1
            centre, anchor, step_cap = _retreat(centre, anchor, step_cap, misses)
            continue
        moved += 1
        frame = camera.capture()
        seen, det = detect_board(cb, dets, frame, wide_seen=eye_to_hand)
        if det is None:
            # a board still in shot is not evidence that the draw reached too far
            if not seen:
                amp = max(amp * AMP_SHRINK, AMP_MIN)
                misses += 1
                centre, anchor, step_cap = _retreat(centre, anchor, step_cap, misses)
            unsolved += int(seen)
            continue
        amp = min(amp * AMP_RECOVER, ceil)
        misses, anchor = 0, centre.copy()
        if R0 is None:
            R0 = det.T_cam_board[:3, :3].copy()
        Q.append(q)
        U.append(_observe(det.T_cam_board, R0))
        if step_cap >= DEPTH_STEP_MIN_M and len(Q) >= DEPTH_FIT_MIN:
            target = _standoff_target(np.array(U)[:, 5], max_poses - tried)
            if target is not None:
                centre = _walk_centre(
                    centre, np.array(Q), np.array(U), home, bounds, step_cap, target
                )
        T_base_flange = robot.get_flange_pose()
        A.append(T_inv(T_base_flange) if eye_to_hand else T_base_flange)
        obs.append(det)
        K = frame.K
        dist = np.resize(np.asarray(frame.dist, dtype=float).ravel(), 5)
    if len(A) < 3 or K is None or dist is None:
        return HandEyeResult(
            nominal.frame,
            nominal.T.copy(),
            False,
            tried,
            moved,
            {"detections": float(len(A))},
            reason="too few detections",
        )
    n_detected = len(A)
    A, obs = _reject_bad_views(A, obs)
    X, Z, rms_px = _solve_pair(A, obs, K, dist)
    pts = np.array([T[:3, 3] for T in _target_pose(A, obs, X)])
    misclose_mm = float(np.mean(np.linalg.norm(pts - pts.mean(axis=0), axis=1)) * 1000)
    err_mm, worst_mm = _probe_error_mm(A, obs, K, dist, X, Z)
    dev_mm = float(np.linalg.norm(X[:3, 3] - nominal.T[:3, 3]) * 1000)
    checks = {
        "probe error": err_mm < PASS_MM,
        "worst resample": worst_mm < CATASTROPHE_MM,
        "misclosure": misclose_mm < MISCLOSE_MM,
        "deviation from nominal": dev_mm < nominal.tol_mm * NOMINAL_TOL_MULT,
        "poses": len(A) >= MIN_POSES,
    }
    failed = [name for name, ok in checks.items() if not ok]
    conf = {
        "detections": float(n_detected),
        "views_used": float(len(A)),
        "seen_unsolved": float(unsolved),
        "probe_err_mm": err_mm,
        "probe_worst_mm": worst_mm,
        "standoff_m": float(np.mean([np.linalg.norm(o.T_cam_board[:3, 3]) for o in obs])),
        "misclosure_mm": misclose_mm,
        "mean_pnp_px": float(np.mean([o.resid_px for o in obs])),
        "refined_rms_px": rms_px,
        "deviation_from_nominal_mm": dev_mm,
    }
    logger.info(
        "Hand-eye self-check",
        passed=not failed,
        failed=failed,
        views=f"{len(A)}/{n_detected}",
        probe_err_mm=round(err_mm, 3),
        misclosure_mm=round(misclose_mm, 3),
        deviation_mm=round(dev_mm, 1),
    )
    return HandEyeResult(nominal.frame, X, not failed, tried, moved, conf, reason=", ".join(failed))
