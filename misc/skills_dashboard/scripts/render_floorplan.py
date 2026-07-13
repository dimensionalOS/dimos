"""Render floorplan.dxf: full sheet + crop around the robot marker."""

import ezdxf
from ezdxf import bbox
from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
import matplotlib.pyplot as plt

BG = "#123252"
INK = "#dce9f5"

doc = ezdxf.readfile("/home/dimos/dimos/floorplan.dxf")
msp = doc.modelspace()

robot_entities = list(msp.query("*[layer=='ROBOT']"))
robot_box = bbox.extents(robot_entities)
print("robot box:", robot_box.extmin, robot_box.extmax)
cx = (robot_box.extmin.x + robot_box.extmax.x) / 2
cy = (robot_box.extmin.y + robot_box.extmax.y) / 2
print("robot center:", cx, cy)

for name, xlim, ylim, size in [
    ("floorplan_detail", (cx - 14, cx + 14), (cy - 9, cy + 9), (12, 7.7)),
]:
    fig = plt.figure(figsize=size, dpi=120)
    ax = fig.add_axes([0, 0, 1, 1])
    ctx = RenderContext(doc)
    backend = MatplotlibBackend(ax)
    cfg = Configuration(
        background_policy=BackgroundPolicy.CUSTOM,
        custom_bg_color=BG,
        color_policy=ColorPolicy.CUSTOM,
        custom_fg_color=INK,
    )
    Frontend(ctx, backend, config=cfg).draw_layout(msp, finalize=True)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    fig.savefig(
        f"/home/dimos/dimos/misc/skills_dashboard/data/01-floorplan/{name}.png",
        dpi=120,
        facecolor=BG,
    )
    plt.close(fig)
    print("wrote", name)
