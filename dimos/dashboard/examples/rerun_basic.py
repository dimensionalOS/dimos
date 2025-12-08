import rerun as rr

from math import tau
import numpy as np
from rerun.utilities import build_color_spiral
from rerun.utilities import bounce_lerp
import rerun.blueprint as rrb

rr.init("rerun_example_dna_abacus")
rr.spawn()
default_blueprint = rrb.Blueprint(
    rrb.Tabs(
        rrb.Spatial3DView(
            name="Spatial3D",
            origin="/spatial3d",
            line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
        ),
        rrb.Spatial2DView(
            name="Spatial2D",
            origin="/color_image",
        ),
    )
)
rr.send_blueprint(default_blueprint)

NUM_POINTS = 100

# Points and colors are both np.array((NUM_POINTS, 3))
points1, colors1 = build_color_spiral(NUM_POINTS)
points2, colors2 = build_color_spiral(NUM_POINTS, angular_offset=tau*0.5)

rr.log("dna/structure/left", rr.Points3D(points1, colors=colors1, radii=0.08))
rr.log("dna/structure/right", rr.Points3D(points2, colors=colors2, radii=0.08))

rr.log(
    "dna/structure/scaffolding",
    rr.LineStrips3D(np.stack((points1, points2), axis=1), colors=[128, 128, 128])
)
offsets = np.random.rand(NUM_POINTS)
beads = [bounce_lerp(points1[n], points2[n], offsets[n]) for n in range(NUM_POINTS)]
colors = [[int(bounce_lerp(80, 230, offsets[n] * 2))] for n in range(NUM_POINTS)]
rr.log(
    "dna/structure/scaffolding/beads",
    rr.Points3D(beads, radii=0.06, colors=np.repeat(colors, 3, axis=-1)),
)