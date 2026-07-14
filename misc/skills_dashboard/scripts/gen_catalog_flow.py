"""Generate the catalog-skill pipeline flow diagram (item 2).

The identify/list/locate skill has grown well past a straight line: a
frame-quality pre-filter, two separate detector instances (always-on
prompt-free + on-demand promptable), depth-or-lidar localization, a dedup DB
that now keeps the sharpest crop per object, VLM refinement of only the
ambiguous ones, optional spatial memory, and the sibling lidar/floorplan
skills that share the same feed. This renders that whole graph as one
theme-matched SVG (light + dark variants, same swap convention as the other
dashboard figures).

Outputs:
  data/02-identify-locate/catalog_flow_{light,dark}.svg

Regenerate: uv run python misc/skills_dashboard/scripts/gen_catalog_flow.py
"""

from __future__ import annotations

from html import escape

OUT = "/home/dimos/dimos/misc/skills_dashboard/data/02-identify-locate"

# Palettes mirror index.html's :root[data-theme] CSS variables so the diagram
# sits flush with the surrounding panel in either theme.
THEMES = {
    "light": dict(
        surface="#ffffff", panel="#eef1f6", ink="#131a24", ink2="#49566a",
        muted="#7d879a", line="#c6cfdd", accent="#2a5fb8", accent_soft="#e3ecf9",
        new="#1baf7a", new_soft="#e2f5ee", arrow="#9aa6b8",
    ),
    "dark": dict(
        surface="#152134", panel="#1a2940", ink="#e8eef7", ink2="#b4c2d6",
        muted="#7d8ca3", line="#33476a", accent="#82abe8", accent_soft="#1c3050",
        new="#2cc48d", new_soft="#123227", arrow="#5a6b86",
    ),
}

W, H = 1180, 340
BW, BH = 156, 52  # default box size

# kind: "io" (rounded, panel fill), "main" (spine), "new" (highlighted addition),
# "opt" (dashed / optional), "skill" (accent fill = agent-callable surface)
# Each node: id -> (cx, cy, title, sub, kind, [w], [h])
NODES = {
    # top lane: on-demand directed search (separate promptable detector)
    "detect":   (300, 52,  "detect([...])", "directed search", "skill"),
    "prompt":   (494, 52,  "promptable YOLO-E", "2nd instance, on demand", "main"),
    # main spine
    "sensors":  (96,  170, "sensors", "/color_image /lidar\n/camera_info /tf", "io", 150, 62),
    "gate":     (300, 170, "frame-quality gate", "drop dark / blurry", "new"),
    "detector": (494, 170, "YOLO-E prompt-free", "open-vocab 2D, per frame", "main"),
    "localize": (688, 170, "localize", "depth | lidar -> world", "main"),
    "db":       (890, 170, "ObjectDB", "dedup + promote\nkeep sharpest crop", "new", 168, 62),
    "refine":   (890, 278, "VLM refine", "ambiguous only (item 5)", "opt", 168),
    "memory":   (688, 278, "spatial memory", "tagged places + recall", "opt"),
    "catalog":  (1084, 170, "catalog_scene()\nlist_observed_items()\nlocate()", "agent answers", "skill", 178, 74),
    # bottom lane: sibling skills that share the same live feed
    "siblings": (300, 278, "generate_floorplan()\nmeasure_space()", "share /lidar + /odom", "skill", 200, 56),
}

# edges: (from, to, [style]) — style "opt" = dashed
EDGES = [
    ("sensors", "gate"),
    ("gate", "detector"),
    ("detector", "localize"),
    ("localize", "db"),
    ("db", "catalog"),
    ("detect", "prompt"),
    ("prompt", "localize", "opt"),      # on-demand pass reuses the localizer
    ("db", "refine", "opt"),
    ("refine", "catalog", "opt"),
    ("memory", "catalog", "opt"),
    ("sensors", "siblings", "opt"),     # feeder also drives the lidar/floorplan skills
]


def _wh(node: tuple) -> tuple[int, int]:
    w = node[5] if len(node) > 5 else BW
    h = node[6] if len(node) > 6 else BH
    return w, h


def _anchor(node: tuple, side: str) -> tuple[float, float]:
    cx, cy = node[0], node[1]
    w, h = _wh(node)
    return {
        "l": (cx - w / 2, cy), "r": (cx + w / 2, cy),
        "t": (cx, cy - h / 2), "b": (cx, cy + h / 2),
    }[side]


def _pick_sides(a: tuple, b: tuple) -> tuple[str, str]:
    """Choose exit/entry sides from relative position (horizontal bias)."""
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    if abs(bx - ax) >= abs(by - ay):
        return ("r", "l") if bx >= ax else ("l", "r")
    return ("b", "t") if by >= ay else ("t", "b")


def _edge_path(a: tuple, b: tuple) -> tuple[float, float, float, float]:
    sa, sb = _pick_sides(a, b)
    x1, y1 = _anchor(a, sa)
    x2, y2 = _anchor(b, sb)
    return x1, y1, x2, y2


def _box(node: tuple, t: dict) -> str:
    cx, cy, title, sub, kind = node[:5]
    w, h = _wh(node)
    x, y = cx - w / 2, cy - h / 2
    fill, stroke, ink, dash = {
        "io":    (t["panel"], t["line"], t["ink"], ""),
        "main":  (t["surface"], t["line"], t["ink"], ""),
        "new":   (t["new_soft"], t["new"], t["ink"], ""),
        "opt":   (t["surface"], t["line"], t["ink2"], "5 4"),
        "skill": (t["accent_soft"], t["accent"], t["ink"], ""),
    }[kind]
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    sw = "2" if kind in ("new", "skill") else "1.2"

    title_lines = title.split("\n")
    sub_lines = sub.split("\n")
    n_title = len(title_lines)
    # vertical layout of title (bold) then sub (muted mono)
    total = n_title * 15 + len(sub_lines) * 12
    ty = cy - total / 2 + 12
    spans = []
    for ln in title_lines:
        spans.append(
            f'<text x="{cx}" y="{ty:.0f}" text-anchor="middle" '
            f'font-size="13" font-weight="600" fill="{ink}">{escape(ln)}</text>'
        )
        ty += 15
    for ln in sub_lines:
        spans.append(
            f'<text x="{cx}" y="{ty:.0f}" text-anchor="middle" font-size="10.5" '
            f'font-family="ui-monospace,Menlo,monospace" fill="{t["muted"]}">{escape(ln)}</text>'
        )
        ty += 12

    badge = ""
    if kind == "new":
        badge = (
            f'<rect x="{x + w - 44:.0f}" y="{y - 9:.0f}" width="40" height="17" rx="8.5" '
            f'fill="{t["new"]}"/>'
            f'<text x="{x + w - 24:.0f}" y="{y + 3:.0f}" text-anchor="middle" font-size="10" '
            f'font-weight="700" fill="{t["surface"]}">NEW</text>'
        )
    return (
        f'<rect x="{x:.0f}" y="{y:.0f}" width="{w}" height="{h}" rx="9" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash_attr}/>'
        + "".join(spans) + badge
    )


def _arrow(a: tuple, b: tuple, t: dict, opt: bool) -> str:
    x1, y1, x2, y2 = _edge_path(a, b)
    stroke = t["arrow"]
    dash = ' stroke-dasharray="5 4"' if opt else ""
    # elbow for non-straight runs so branch lines read cleanly
    if abs(x1 - x2) > 2 and abs(y1 - y2) > 2:
        sa, _ = _pick_sides(a, b)
        if sa in ("r", "l"):
            mid = f"L {x2:.0f} {y1:.0f} "
        else:
            mid = f"L {x1:.0f} {y2:.0f} "
        d = f"M {x1:.0f} {y1:.0f} {mid}L {x2:.0f} {y2:.0f}"
    else:
        d = f"M {x1:.0f} {y1:.0f} L {x2:.0f} {y2:.0f}"
    return (
        f'<path d="{d}" fill="none" stroke="{stroke}" stroke-width="1.6"{dash} '
        f'marker-end="url(#arw)"/>'
    )


def render(theme: str) -> str:
    t = THEMES[theme]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
        f'font-family="ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif">',
        f'<defs><marker id="arw" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" '
        f'markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{t["arrow"]}"/></marker></defs>',
        f'<rect x="0" y="0" width="{W}" height="{H}" fill="{t["surface"]}"/>',
        # lane captions — placed in the gaps above each row so they never
        # collide with the left-most box in that lane
        f'<text x="14" y="18" font-size="11" font-weight="700" fill="{t["muted"]}" '
        f'letter-spacing="0.5">ON DEMAND</text>',
        f'<text x="14" y="126" font-size="11" font-weight="700" fill="{t["muted"]}" '
        f'letter-spacing="0.5">ALWAYS-ON CATALOG PIPELINE</text>',
        f'<text x="14" y="234" font-size="11" font-weight="700" fill="{t["muted"]}" '
        f'letter-spacing="0.5">OPTIONAL / SIBLING SKILLS</text>',
    ]
    for e in EDGES:
        opt = len(e) > 2 and e[2] == "opt"
        parts.append(_arrow(NODES[e[0]], NODES[e[1]], t, opt))
    for node in NODES.values():
        parts.append(_box(node, t))
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    for theme in ("light", "dark"):
        svg = render(theme)
        path = f"{OUT}/catalog_flow_{theme}.svg"
        with open(path, "w") as f:
            f.write(svg)
        print(f"wrote {path} ({len(svg)} bytes)")


if __name__ == "__main__":
    main()
