"""Figure for the dashboard: the voxel model + cut-plane slicing, on real data.

Recreates (statically) what `demo_lidar_floorplan.py --viewer` shows live in
dimos-viewer: the voxel model of chinaOffice.rrd, the sweeping cut slab, and
the horizontal section at that height. Light + dark variants.
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.mapping.reconstruction.scene_model import SceneModel

OUT = "/home/dimos/dimos/misc/skills_dashboard/data/01-floorplan"

THEMES = {
    "light": dict(surface="#ffffff", ink="#131a24", ink2="#49566a", muted="#7d879a",
                  grid="#dde3ec", blue="#2a78d6", red="#e34948"),
    "dark": dict(surface="#152134", ink="#e8eef7", ink2="#b4c2d6", muted="#7d8ca3",
                 grid="#24344e", blue="#5f9be8", red="#e66767"),
}

model = SceneModel.from_rrd("/home/dimos/dimos/chinaOffice.rrd")
lo = np.percentile(model.points, 0.5, axis=0) - 0.3
hi = np.percentile(model.points, 99.5, axis=0) + 0.3
vox = model.voxel_occupancy(voxel=0.15, bounds=(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]))

Z_CUT = 1.6  # level 1, wall height
k = vox.layer_index(Z_CUT)
occ = vox.occupancy(2)
kk, rr, cc = np.nonzero(occ)
pick = np.random.default_rng(0).choice(len(kk), min(60_000, len(kk)), replace=False)
kk, rr, cc = kk[pick], rr[pick], cc[pick]
x = vox.origin[0] + (cc + 0.5) * vox.voxel
y = vox.origin[1] + (rr + 0.5) * vox.voxel
z = vox.origin[2] + (kk + 0.5) * vox.voxel

for theme, t in THEMES.items():
    fig = plt.figure(figsize=(11.6, 4.6), dpi=115)
    fig.patch.set_facecolor(t["surface"])

    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.set_facecolor(t["surface"])
    in_slab = np.abs(z - Z_CUT) < vox.voxel
    ax1.scatter(x[~in_slab], y[~in_slab], z[~in_slab], s=0.4, c=z[~in_slab],
                cmap="Blues_r" if theme == "light" else "Blues", alpha=0.35, linewidths=0)
    ax1.scatter(x[in_slab], y[in_slab], z[in_slab], s=1.6, color=t["red"], linewidths=0)
    xx, yy = np.meshgrid([x.min(), x.max()], [y.min(), y.max()])
    ax1.plot_surface(xx, yy, np.full_like(xx, Z_CUT), alpha=0.15, color=t["blue"])
    ax1.set_title(f"voxel model + cut plane at z={Z_CUT} m", fontsize=10, color=t["ink"])
    ax1.set_box_aspect((np.ptp(x), np.ptp(y), 3 * np.ptp(z)))
    ax1.axis("off")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_facecolor(t["surface"])
    layer = np.log1p(vox.counts[k].astype(np.float32))
    ax2.imshow(layer, origin="lower", cmap="Blues" if theme == "light" else "Blues_r",
               interpolation="nearest")
    ax2.set_title("the section at that height (viewer's 2D pane)", fontsize=10, color=t["ink"])
    ax2.tick_params(colors=t["muted"], labelsize=7)
    for s in ax2.spines.values():
        s.set_color(t["grid"])

    fig.tight_layout(pad=1.2)
    fig.savefig(f"{OUT}/voxel_slicing_{theme}.png", dpi=115, facecolor=t["surface"])
    plt.close(fig)
    print("wrote", theme)
