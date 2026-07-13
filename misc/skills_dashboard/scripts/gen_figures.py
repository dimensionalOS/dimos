"""Generate dashboard figures by running the real skill code on synthetic scenes.

Outputs (light + dark variants each):
  lidar_localization_{light,dark}.png  — item 2: Object.from_2d_to_list_lidar demo
  lidar_accumulation_{light,dark}.png  — item 3: LidarPointCloudClient demo
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.mapping.pointclouds.live import LidarPointCloudClient
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import Object

OUT2 = "/home/dimos/dimos/misc/skills_dashboard/data/02-identify-locate"
OUT3 = "/home/dimos/dimos/misc/skills_dashboard/data/03-lidar-to-numpy"

THEMES = {
    "light": dict(
        surface="#ffffff", ink="#131a24", ink2="#49566a", muted="#7d879a",
        grid="#dde3ec", blue="#2a78d6", aqua="#1baf7a", red="#e34948",
        cloud="#b9c4d6",
    ),
    "dark": dict(
        surface="#152134", ink="#e8eef7", ink2="#b4c2d6", muted="#7d8ca3",
        grid="#24344e", blue="#5f9be8", aqua="#2cc48d", red="#e66767",
        cloud="#3d5273",
    ),
}


def style_ax(ax, t):
    ax.set_facecolor(t["surface"])
    for s in ax.spines.values():
        s.set_color(t["grid"])
    ax.tick_params(colors=t["muted"], labelsize=8)
    ax.xaxis.label.set_color(t["ink2"])
    ax.yaxis.label.set_color(t["ink2"])
    ax.title.set_color(t["ink"])
    ax.grid(color=t["grid"], linewidth=0.6, alpha=0.7)
    ax.set_axisbelow(True)


# ------------------------------------------------------------------ item 2 --
# Scene in optical convention (x right, y down, z forward), world == optical
# via identity transform — same setup as the unit test, extended to a fuller
# scene: two object blobs + wall + floor clutter.
rng = np.random.default_rng(7)
cup = np.array([-0.55, 0.25, 1.8]) + rng.normal(0, 0.035, (160, 3))
chair = np.array([0.62, 0.30, 2.6]) + rng.normal(0, 0.10, (260, 3)) * [1.0, 1.6, 1.0]
wall = np.column_stack([
    rng.uniform(-2.2, 2.2, 900), rng.uniform(-1.2, 0.9, 900), rng.normal(3.6, 0.03, 900)
])
floor = np.column_stack([
    rng.uniform(-2.2, 2.2, 500), rng.normal(0.95, 0.02, 500), rng.uniform(0.8, 3.6, 500)
])
scene = np.vstack([cup, chair, wall, floor])
world_pc = PointCloud2.from_numpy(scene.astype(float), frame_id="world", timestamp=1.0)

camera_info = CameraInfo.from_intrinsics(
    fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera_optical"
)
img = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8),
                       format=ImageFormat.BGR, frame_id="camera_optical", ts=1.0)


def project(pts):
    z = pts[:, 2]
    u = 500.0 * pts[:, 0] / z + 320.0
    v = 500.0 * pts[:, 1] / z + 240.0
    return u, v, z


def bbox_of(pts, pad=8):
    u, v, _ = project(pts)
    return (u.min() - pad, v.min() - pad, u.max() + pad, v.max() + pad)


dets = [
    Detection2DBBox(bbox=bbox_of(cup), track_id=1, class_id=0, confidence=0.91,
                    name="cup", ts=1.0, image=img),
    Detection2DBBox(bbox=bbox_of(chair), track_id=2, class_id=1, confidence=0.84,
                    name="chair", ts=1.0, image=img),
]
objects = Object.from_2d_to_list_lidar(
    detections_2d=ImageDetections2D(img, dets),
    world_pointcloud=world_pc,
    camera_info=camera_info,
    world_to_optical_transform=Transform.identity(),
    filters=None,  # default stack: raycast + radius_outlier + statistical
)
print("localized:", [(o.name, round(o.center.x, 2), round(o.center.y, 2), round(o.center.z, 2))
                     for o in objects])
truth = {"cup": np.array([-0.55, 0.25, 1.8]), "chair": np.array([0.62, 0.30, 2.6])}
for o in objects:
    err = np.linalg.norm(np.array([o.center.x, o.center.y, o.center.z]) - truth[o.name])
    print(f"  {o.name}: |err| = {err:.3f} m")

for theme, t in THEMES.items():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.4), dpi=115)
    fig.patch.set_facecolor(t["surface"])

    # -- image plane view
    u, v, z = project(scene)
    inside = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
    ax1.scatter(u[inside], v[inside], s=2.2, c=z[inside], cmap="Blues_r" if theme == "light" else "Blues",
                alpha=0.75, linewidths=0)
    colors = {"cup": t["blue"], "chair": t["aqua"]}
    for det in dets:
        x1, y1, x2, y2 = det.bbox
        c = colors[det.name]
        ax1.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                                    edgecolor=c, linewidth=1.8))
        ax1.text(x1, y1 - 7, f"{det.name} {det.confidence:.2f}", color=c,
                 fontsize=9, fontweight="bold", family="monospace")
    ax1.set_xlim(0, 640); ax1.set_ylim(480, 0)
    ax1.set_title("camera view — YOLO bboxes + projected lidar", fontsize=10)
    ax1.set_xlabel("u (px)"); ax1.set_ylabel("v (px)")
    style_ax(ax1, t)

    # -- top-down world view
    ax2.scatter(scene[:, 0], scene[:, 2], s=2.2, color=t["cloud"], alpha=0.7,
                linewidths=0, label="lidar cloud")
    for o in objects:
        c = colors[o.name]
        pts, _ = o.pointcloud.as_numpy()
        ax2.scatter(pts[:, 0], pts[:, 2], s=3.5, color=c, linewidths=0)
        ax2.scatter([o.center.x], [o.center.z], marker="x", s=90, color=c, linewidths=2.4)
        ax2.add_patch(plt.Rectangle((o.center.x - o.size.x / 2, o.center.z - o.size.z / 2),
                                    o.size.x, o.size.z, fill=False, edgecolor=c,
                                    linewidth=1.4, linestyle=(0, (3, 2))))
        gt = truth[o.name]
        err = np.linalg.norm(np.array([o.center.x, o.center.y, o.center.z]) - gt)
        ax2.annotate(f"{o.name}\n±{err:.2f} m", (o.center.x, o.center.z),
                     xytext=(10, 8), textcoords="offset points", color=c,
                     fontsize=8.5, fontweight="bold", family="monospace")
    # camera FOV
    for sx in (-0.64, 0.64):
        ax2.plot([0, sx * 4.2], [0, 4.2], color=t["muted"], linewidth=0.9,
                 linestyle=(0, (2, 3)))
    ax2.scatter([0], [0], marker="^", s=70, color=t["ink"], zorder=5)
    ax2.text(0.08, 0.02, "camera", color=t["muted"], fontsize=8.5, family="monospace")
    ax2.set_xlim(-2.4, 2.4); ax2.set_ylim(-0.25, 4.0)
    ax2.set_title("world frame (top-down) — localized objects", fontsize=10)
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("z, depth (m)")
    style_ax(ax2, t)
    ax2.set_aspect("equal", adjustable="box")

    fig.tight_layout(pad=1.4)
    fig.savefig(f"{OUT2}/lidar_localization_{theme}.png", dpi=115, facecolor=t["surface"])
    plt.close(fig)
    print("wrote lidar_localization_" + theme)

# ------------------------------------------------------------------ item 3 --
# Spin a synthetic mid360-ish scanner around a 6x4 m room and feed the real
# client callbacks, recording snapshot growth exactly as a consumer would.
client = LidarPointCloudClient()
rng = np.random.default_rng(3)

W, H = 6.0, 4.0
walls = []
n_wall = 3200
xs = rng.uniform(-W / 2, W / 2, n_wall)
ys = rng.uniform(-H / 2, H / 2, n_wall)
side = rng.integers(0, 4, n_wall)
wx = np.where(side < 2, np.where(side == 0, -W / 2, W / 2), xs)
wy = np.where(side < 2, ys, np.where(side == 2, -H / 2, H / 2))
wz = rng.uniform(0.0, 2.4, n_wall)
room = np.column_stack([wx, wy, wz])
pillar = np.column_stack([
    1.2 + 0.18 * np.cos(a := rng.uniform(0, 2 * np.pi, 350)),
    -0.6 + 0.18 * np.sin(a), rng.uniform(0, 2.0, 350)
])
env = np.vstack([room, pillar])

counts = [0]
n_msgs = 48
for i in range(n_msgs):
    ang = 2 * np.pi * i / n_msgs
    bearing = np.arctan2(env[:, 1], env[:, 0])
    d = np.angle(np.exp(1j * (bearing - ang)))
    sector = np.abs(d) < np.deg2rad(38)
    pts = env[sector] + rng.normal(0, 0.008, (sector.sum(), 3))
    msg = PointCloud2.from_numpy(pts.astype(float), frame_id="world", timestamp=float(i))
    client._on_lidar(msg)
    counts.append(len(client.snapshot()))

snap = client.snapshot()
print("final snapshot:", snap.shape, snap.dtype, "messages:", client.message_count)

for theme, t in THEMES.items():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.2), dpi=115,
                                   gridspec_kw={"width_ratios": [1.15, 1]})
    fig.patch.set_facecolor(t["surface"])

    ax1.fill_between(range(len(counts)), counts, color=t["blue"], alpha=0.14, step="post")
    ax1.step(range(len(counts)), counts, where="post", color=t["blue"], linewidth=2.0)
    ax1.scatter([len(counts) - 1], [counts[-1]], color=t["blue"], s=34, zorder=5)
    ax1.annotate(f"{counts[-1]:,} pts", (len(counts) - 1, counts[-1]),
                 xytext=(-8, -16), textcoords="offset points", ha="right",
                 color=t["ink"], fontsize=10, fontweight="bold", family="monospace")
    ax1.set_title("snapshot() size while a scanner spins one revolution", fontsize=10)
    ax1.set_xlabel("/lidar messages received"); ax1.set_ylabel("points in snapshot()")
    ax1.set_xlim(0, len(counts) - 1); ax1.set_ylim(0, counts[-1] * 1.12)
    style_ax(ax1, t)

    ax2.scatter(snap[:, 0], snap[:, 1], s=2.0, c=snap[:, 2],
                cmap="Blues_r" if theme == "light" else "Blues", alpha=0.8, linewidths=0)
    ax2.scatter([0], [0], marker="^", s=70, color=t["ink"], zorder=5)
    ax2.text(0.1, -0.28, "lidar", color=t["muted"], fontsize=8.5, family="monospace")
    ax2.set_title("accumulated (N, 3) cloud, top-down (z as shade)", fontsize=10)
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.set_aspect("equal", adjustable="box")
    style_ax(ax2, t)

    fig.tight_layout(pad=1.4)
    fig.savefig(f"{OUT3}/lidar_accumulation_{theme}.png", dpi=115, facecolor=t["surface"])
    plt.close(fig)
    print("wrote lidar_accumulation_" + theme)
