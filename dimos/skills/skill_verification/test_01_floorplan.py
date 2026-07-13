# Copyright 2025-2026 Dimensional Inc.
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

"""Wishlist item 1 — architectural floorplan generation.

Real-data tier: the floorplan pipeline has no synthetic proxy, so this runs the
real generator on a recorded mapping session (chinaOffice.rrd by default, or
$SKILL_VERIFY_RRD) with AI passes off — the DXF is *generated from the .rrd*, no
existing drawing needed — and writes the DXF + JPEG sheet + a DXF render for
review. Skips cleanly when no recording (or ezdxf) is present, so it runs
locally where the sample recordings live and is a no-op in CI.
See manifest.yaml (01-floorplan).
"""

from __future__ import annotations

import shutil

import pytest

# No self_hosted marker: this gates on *data presence* instead. It runs wherever
# a sample recording + ezdxf are available (locally), and skips cleanly in CI /
# fresh checkouts where the (large, gitignored) recording isn't present.

SKILL_ID = "01-floorplan"

# The drawing must actually place wall geometry on this layer, not just define it.
_WALL_LAYER = "A-WALL"


def test_generate_floorplan_from_recording(skill_output, recording_path) -> None:
    """generate_floorplan() on a real recording yields a layered DXF + JPEG sheet."""
    if recording_path is None:
        pytest.skip(
            "no recording available — set SKILL_VERIFY_RRD or place chinaOffice.rrd at the repo root"
        )

    # ezdxf backs the floorplan generator's DXF output and this test's DXF render;
    # it is an optional/heavy dep, so skip cleanly rather than error when absent.
    pytest.importorskip("ezdxf", reason="floorplan generation needs ezdxf")

    import ezdxf
    from ezdxf.addons.drawing import Frontend, RenderContext
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    import matplotlib.pyplot as plt

    from dimos.mapping.floorplan.generator import FloorplanOptions, generate_floorplan

    opts = FloorplanOptions(
        rrd=recording_path,
        ai_review=False,  # explicit off: no OPENAI_API_KEY needed
        ai_render=None,
        out=skill_output.dir / "floorplan",
        project="SKILL VERIFICATION",
    )
    result = generate_floorplan(opts)

    # --- machine-checkable invariants -------------------------------------
    assert result.dxf.is_file() and result.dxf.stat().st_size > 0
    assert result.jpeg.is_file() and result.jpeg.stat().st_size > 0
    assert result.sheets, "no floor levels produced"

    doc = ezdxf.readfile(str(result.dxf))
    msp = doc.modelspace()
    assert len(list(msp)) > 0, "empty DXF modelspace"
    wall_entities = list(msp.query(f'*[layer=="{_WALL_LAYER}"]'))
    assert wall_entities, f"no wall geometry drawn on {_WALL_LAYER}"

    # --- review artifacts -------------------------------------------------
    dxf_out = skill_output.path("floorplan.dxf")
    if result.dxf != dxf_out:
        shutil.copy(result.dxf, dxf_out)
    skill_output.produced("floorplan.dxf")

    jpg_out = skill_output.path("floorplan_sheet.jpg")
    if result.jpeg != jpg_out:
        shutil.copy(result.jpeg, jpg_out)
    skill_output.produced("floorplan_sheet.jpg")

    # Render the DXF to a PNG so the layers are viewable without a CAD tool.
    fig = plt.figure(figsize=(12, 8), dpi=120)
    ax = fig.add_axes([0, 0, 1, 1])
    Frontend(RenderContext(doc), MatplotlibBackend(ax)).draw_layout(msp, finalize=True)
    fig.savefig(skill_output.produced("floorplan_render.png"), dpi=120)
    plt.close(fig)
