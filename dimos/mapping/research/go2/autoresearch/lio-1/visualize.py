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

"""Render a trajectory visualization for the current run and save a small,
commit-friendly downsampled trajectory. Called automatically at the end of
train.py (non-fatal), or run standalone: `uv run visualize.py`.

Outputs (overwritten each run; commit them on a "keep" so git history holds the
per-experiment version):
  - viz.png        — top-down xy (LIO rigid-aligned to GT) + position error vs time
  - traj_ds.tsv    — downsampled raw LIO trajectory (t x y z), ~2k poses, ~60 KB

PNG (not JPEG): the plot is thin-line art + text, which PNG stores smaller and
without the ringing artifacts JPEG adds to lines.
"""

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import prepare

DS_POSES = 2000  # downsample target for the saved trajectory + plot
FIT_WINDOW = 30.0  # seconds; the second top-down panel aligns only on the early
# window (where LIO still tracks) to show where it peels off
VIZ_PNG = "viz.png"
TRAJ_DS = "traj_ds.tsv"


def render(traj_path=prepare.TRAJ_PATH, out_png=VIZ_PNG, out_traj=TRAJ_DS):
    gt_t, gt_p = prepare.load_gt()
    t, P = prepare.load_traj(traj_path)

    # --- save downsampled raw trajectory (commit-friendly) ---
    step = max(1, len(t) // DS_POSES)
    ds = np.column_stack([t[::step], P[::step]])
    np.savetxt(
        out_traj,
        ds,
        fmt="%.5f",
        delimiter="\t",
        header="downsampled raw LIO trajectory\nt_rel\tx\ty\tz",
    )

    # --- 2D rigid-align LIO onto GT: full-run (the metric) and first-FIT_WINDOW s ---
    gt_xy, xy = gt_p[:, :2], P[:, :2]
    lo, hi = max(t[0], gt_t[0]), min(t[-1], gt_t[-1])
    ov = (t >= lo) & (t <= hi)
    tv, xyv = t[ov], xy[ov]
    gti = np.column_stack([np.interp(tv, gt_t, gt_xy[:, k]) for k in range(2)])

    def align(fit_mask):
        R, tr = prepare._umeyama_2d(xyv[fit_mask], gti[fit_mask])
        al = (R @ xyv.T).T + tr
        return al, np.linalg.norm(al - gti, axis=1)

    early = tv <= (tv[0] + FIT_WINDOW)
    al_early, err_early = align(early)
    al_full, err_full = align(np.ones(len(tv), bool))
    ate = float(np.sqrt(np.mean(err_full**2)))  # the metric: standard full-run ATE

    def topdown(a, al, title):
        a.plot(gt_xy[:, 0], gt_xy[:, 1], "k-", lw=2, label="robot_odom (gt)", zorder=5)
        a.plot(al[:, 0], al[:, 1], "tab:blue", lw=1, alpha=0.85, label="LIO (aligned)")
        a.plot(*gti[0], "go", ms=9, zorder=6)
        a.plot(*gti[-1], "kx", ms=11, mew=3, zorder=6)
        cx, cy = gt_xy[:, 0].mean(), gt_xy[:, 1].mean()
        r = (
            max(gt_xy[:, 0].max() - gt_xy[:, 0].min(), gt_xy[:, 1].max() - gt_xy[:, 1].min()) * 0.75
            + 1.5
        )
        a.set_xlim(cx - r, cx + r)
        a.set_ylim(cy - r, cy + r)
        a.set_aspect("equal")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
        a.set_title(title)
        a.set_xlabel("x (m)")
        a.set_ylabel("y (m)")

    fig, ax = plt.subplots(1, 3, figsize=(19, 5.5))
    topdown(
        ax[0],
        al_early,
        f"Top-down xy — aligned on first {FIT_WINDOW:.0f}s\n(early tracking, then where it peels off)",
    )
    topdown(ax[1], al_full, "Top-down xy — full-run aligned (ATE metric)\ngreen=start  kx=end")
    ax[2].plot(tv, err_early, "tab:green", lw=1.2, label=f"first-{FIT_WINDOW:.0f}s align")
    ax[2].plot(tv, err_full, "tab:blue", lw=1.2, label="full-run align")
    ax[2].axvline(FIT_WINDOW, color="gray", ls=":", lw=1)
    ax[2].set_title(f"position error vs gt   (val_ate_xy = {ate:.3f} m)")
    ax[2].set_xlabel("t (s)")
    ax[2].set_ylabel("|p - gt| (m)")
    ax[2].grid(alpha=0.3)
    ax[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=100)
    plt.close(fig)
    return ate


if __name__ == "__main__":
    ate = render()
    print(f"wrote {VIZ_PNG} + {TRAJ_DS}  (val_ate_xy={ate:.3f})")
