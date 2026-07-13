"""Generate item 8 dashboard figures by running the real SceneModel code.

A synthetic two-story house (floor slabs, walls, a table, a stair ramp) is
built as a raw point cloud, fed through `SceneModel`, and rendered as:
  left  — 3D scatter of the model with the two section planes indicated
  mid   — horizontal_section() plan cut through story 1's walls
  right — vertical_section() elevation across both stories

Outputs: scene_model_{light,dark}.png in data/08-scene-3d-model/.
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.mapping.reconstruction.scene_model import SceneModel

OUT = "/home/dimos/dimos/misc/skills_dashboard/data/08-scene-3d-model"

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


# ---------------------------------------------------------------- scene --
rng = np.random.default_rng(11)
pts = []


def plane(xs, ys, zs, n):
    pts.append(np.column_stack(
        [rng.uniform(*xs, n), rng.uniform(*ys, n), rng.uniform(*zs, n)]
    ))


for floor_z in (0.0, 3.0):  # two stories
    plane((0, 8), (0, 5), (floor_z, floor_z + 0.03), 6000)  # slab
    plane((0, 8), (0, 0.06), (floor_z, floor_z + 2.9), 3500)  # south wall
    plane((0, 8), (4.94, 5), (floor_z, floor_z + 2.9), 3500)  # north wall
    plane((0, 0.06), (0, 5), (floor_z, floor_z + 2.9), 2500)  # west wall
    plane((7.94, 8), (0, 5), (floor_z, floor_z + 2.9), 2500)  # east wall
plane((0, 8), (0, 5), (5.95, 6.0), 5000)  # roof
plane((2.2, 3.2), (1.6, 2.6), (0.72, 0.78), 700)  # table, story 1
plane((5.2, 6.4), (2.0, 2.4), (0.0, 3.0), 900)  # stair volume between stories

model = SceneModel(np.vstack(pts).astype(np.float32))
plan = model.horizontal_section(z=1.5, thickness=0.2, resolution=0.05)
elev = model.vertical_section((0, 2.1), (8, 2.1), thickness=0.5, resolution=0.05)

for theme, t in THEMES.items():
    fig = plt.figure(figsize=(13.2, 4.4), facecolor=t["surface"])
    ax3 = fig.add_subplot(1, 3, 1, projection="3d")
    ax_p = fig.add_subplot(1, 3, 2)
    ax_e = fig.add_subplot(1, 3, 3)

    sub = model.points[rng.choice(len(model.points), 9000, replace=False)]
    ax3.scatter(sub[:, 0], sub[:, 1], sub[:, 2], s=0.4, c=sub[:, 2],
                cmap="viridis", linewidths=0)
    xx, yy = np.meshgrid([0, 8], [0, 5])
    ax3.plot_surface(xx, yy, np.full_like(xx, 1.5), alpha=0.18, color=t["blue"])
    ax3.plot([0, 8], [2.1, 2.1], [0, 0], color=t["red"], lw=2)
    ax3.set_facecolor(t["surface"])
    ax3.set_title("SceneModel (9k of 42k pts)", color=t["ink"], fontsize=10)
    ax3.tick_params(colors=t["muted"], labelsize=6)
    ax3.view_init(elev=18, azim=-60)

    ax_p.imshow(np.log1p(plan.density), origin="lower", cmap="Greys" if theme == "light"
                else "Blues", extent=(plan.extent[0], plan.extent[2],
                                      plan.extent[1], plan.extent[3]))
    ax_p.set_title("horizontal_section(z=1.5)", fontsize=10)
    ax_p.set_xlabel("x [m]")
    ax_p.set_ylabel("y [m]")
    style_ax(ax_p, t)

    ax_e.imshow(np.log1p(elev.density), origin="lower", cmap="Greys" if theme == "light"
                else "Blues", extent=(elev.extent[0], elev.extent[2],
                                      elev.extent[1], elev.extent[3]))
    ax_e.set_title("vertical_section((0,2.1)→(8,2.1))", fontsize=10)
    ax_e.set_xlabel("along cut [m]")
    ax_e.set_ylabel("z [m]")
    style_ax(ax_e, t)

    fig.tight_layout()
    fig.savefig(f"{OUT}/scene_model_{theme}.png", dpi=150, facecolor=t["surface"])
    plt.close(fig)
    print(f"wrote {OUT}/scene_model_{theme}.png")
