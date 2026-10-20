#!/usr/bin/env python3
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

"""Render a compact top-down audit HTML for a localize-backed VQA dataset.

Base layer: recording ``global_costmap``. Overlays: verified localize object
centroids (from full audit) + per-frame robot pose/yaw + published questions.
"""

from __future__ import annotations

import argparse
import base64
import html
import io
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

from dimos.evals.vqa.localize_coverage import case_family_from_id
from dimos.evals.vqa.localize_recording import robot_pose_xyz_yaw
from dimos.memory.cli.dataset import open_dataset
from dimos.perception.localize.rig import Rig, window_tf

# Distinct, muted categorical colors (colorblind-friendlier than rainbow).
_CAT_COLORS = [
    "#1b9e77",
    "#d95f02",
    "#7570b3",
    "#e7298a",
    "#66a61e",
    "#e6ab02",
    "#a6761d",
    "#666666",
    "#1f78b4",
    "#b15928",
    "#6a3d9a",
    "#33a02c",
]


def _family(case_id: str) -> str:
    return case_family_from_id(case_id)


def _obj_key(name: str, x: float, y: float) -> tuple[object, ...]:
    return (round(x, 1), round(y, 1), name.casefold())


def _collect_objects(full_audit: Path) -> list[dict[str, object]]:
    objs: dict[tuple[object, ...], dict[str, object]] = {}
    for gt_path in sorted(full_audit.glob("frame-*/ground_truth.json")):
        for row in json.loads(gt_path.read_text()):
            if row.get("status") != "answered":
                continue
            evidence = (row.get("answer") or {}).get("evidence") or {}
            if not isinstance(evidence, dict) or "position_world_xyz" not in evidence:
                continue
            xyz = tuple(float(v) for v in evidence["position_world_xyz"][:3])
            name = str(evidence.get("category") or evidence.get("object_name") or "?")
            key = _obj_key(name, xyz[0], xyz[1])
            objs[key] = {"name": name, "x": xyz[0], "y": xyz[1], "z": xyz[2], "key": key}
    return sorted(objs.values(), key=lambda row: (str(row["name"]).casefold(), row["x"], row["y"]))


def _evidence_for_question(
    full_audit: Path, frame: str, question: str
) -> dict[str, object] | None:
    gt_path = full_audit / frame / "ground_truth.json"
    if not gt_path.exists():
        return None
    for row in json.loads(gt_path.read_text()):
        if row.get("status") != "answered":
            continue
        answer = row.get("answer") or {}
        if answer.get("question") != question:
            continue
        evidence = answer.get("evidence") or {}
        return evidence if isinstance(evidence, dict) else None
    return None


def _frame_object_ids(
    full_audit: Path,
    frame: str,
    key_to_index: dict[tuple[object, ...], int],
) -> list[int]:
    """All localize instance centroids mentioned in this frame's audit answers."""
    gt_path = full_audit / frame / "ground_truth.json"
    if not gt_path.exists():
        return []
    ids: list[int] = []
    seen: set[int] = set()
    for row in json.loads(gt_path.read_text()):
        if row.get("status") != "answered":
            continue
        evidence = (row.get("answer") or {}).get("evidence") or {}
        if not isinstance(evidence, dict) or "position_world_xyz" not in evidence:
            continue
        xyz = tuple(float(v) for v in evidence["position_world_xyz"][:3])
        name = str(evidence.get("category") or evidence.get("object_name") or "?")
        index = key_to_index.get(_obj_key(name, xyz[0], xyz[1]))
        if index is None or index in seen:
            continue
        seen.add(index)
        ids.append(index)
    return ids


def _collect_frame_meta(
    dataset: Path,
    full_audit: Path,
    objects: list[dict[str, object]],
) -> tuple[
    list[dict[str, object]],
    dict[str, tuple[tuple[float, float, float], float]],
    dict[str, list[int]],
]:
    cases = [
        json.loads(line)
        for line in (dataset / "cases.jsonl").read_text().splitlines()
        if line.strip()
    ]
    labels = {
        row["id"]: row["answer"]
        for row in (
            json.loads(line)
            for line in (dataset / "labels.jsonl").read_text().splitlines()
            if line.strip()
        )
    }
    key_to_index = {obj["key"]: index for index, obj in enumerate(objects, start=1)}
    index_category = {
        index: str(obj["name"]).casefold() for index, obj in enumerate(objects, start=1)
    }

    robots: dict[str, tuple[tuple[float, float, float], float]] = {}
    for gt_path in full_audit.glob("frame-*/ground_truth.json"):
        frame = gt_path.parent.name
        for row in json.loads(gt_path.read_text()):
            if row.get("status") != "answered":
                continue
            evidence = (row.get("answer") or {}).get("evidence") or {}
            if not isinstance(evidence, dict):
                continue
            if "robot_position_xyz" in evidence and "robot_yaw_rad" in evidence:
                xyz = tuple(float(v) for v in evidence["robot_position_xyz"][:3])
                robots[frame] = (xyz, float(evidence["robot_yaw_rad"]))  # type: ignore[assignment]
                break

    frames_needed = {f"frame-{case['id'].split('-')[1]}" for case in cases}
    frame_objs = {
        frame: _frame_object_ids(full_audit, frame, key_to_index) for frame in frames_needed
    }

    questions: list[dict[str, object]] = []
    for case in cases:
        frame = f"frame-{case['id'].split('-')[1]}"
        family = _family(case["id"])
        evidence = _evidence_for_question(full_audit, frame, case["question"])
        obj_ids: list[int] = []
        if evidence and "position_world_xyz" in evidence:
            xyz = tuple(float(v) for v in evidence["position_world_xyz"][:3])
            name = str(evidence.get("category") or evidence.get("object_name") or "?")
            index = key_to_index.get(_obj_key(name, xyz[0], xyz[1]))
            if index is not None:
                obj_ids.append(index)
        elif evidence and family == "ego_any_fore_aft":
            category = str(evidence.get("category") or evidence.get("object_name") or "").casefold()
            obj_ids = [
                index
                for index in frame_objs.get(frame, [])
                if index_category.get(index) == category
            ]
        elif family == "height_order" and evidence:
            # Prefer explicit xyz rows if present; else leave empty (frame fallback).
            pass
        questions.append(
            {
                "id": case["id"],
                "frame": frame,
                "family": family,
                "question": case["question"],
                "answer": labels[case["id"]],
                "choices": case["choices"],
                "obj_ids": obj_ids,
            }
        )
    return questions, robots, frame_objs


def _fill_missing_poses(
    recording: Path,
    dataset: Path,
    robots: dict[str, tuple[tuple[float, float, float], float]],
    frames: set[str],
) -> None:
    missing = sorted(frames - set(robots))
    if not missing:
        return
    timestamps: list[float] = []
    frame_ts: dict[str, float] = {}
    for frame in missing:
        meta = json.loads((dataset / "audit" / frame / "frame.json").read_text())
        ts = float(meta["timestamp"])
        frame_ts[frame] = ts
        timestamps.append(ts)
    store = open_dataset(str(recording))
    after = min(timestamps) - 2.0
    before = max(timestamps) + 2.0
    tf = window_tf(store, after, before)
    # cameras unused; only need tf + world frame for base_link lookup
    rig = Rig(cameras={}, tf=tf)
    for frame, ts in frame_ts.items():
        pose = robot_pose_xyz_yaw(rig, ts)
        if pose is None:
            continue
        robots[frame] = pose


def _costmap_png(recording: Path) -> tuple[bytes, object]:
    store = open_dataset(str(recording))
    grid_msg = store.streams.global_costmap.last().data
    grid = np.asarray(grid_msg.grid, dtype=np.int16)
    # Map: unknown=#eee, free=#fff, occupied=dark by cost.
    rgba = np.empty((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
    rgba[:] = 238
    free = grid == 0
    rgba[free] = 255
    occ = grid > 0
    # Higher cost → darker.
    costs = np.clip(grid[occ].astype(np.float32), 0, 100)
    darkness = (40 + (costs / 100.0) * 170).astype(np.uint8)
    rgba[occ, 0] = 255 - darkness
    rgba[occ, 1] = 255 - darkness
    rgba[occ, 2] = 255 - darkness
    # Flip vertically so +Y world points up on screen (grid row 0 is usually min Y).
    image = PILImage.fromarray(rgba[::-1], mode="RGB")
    buf = io.BytesIO()
    image.save(buf, format="PNG", optimize=True)
    return buf.getvalue(), grid_msg


def _world_to_px(
    x: float,
    y: float,
    grid_msg: object,
) -> tuple[float, float]:
    cell = grid_msg.world_to_grid((x, y, 0.0))
    # After vertical flip: px_y = (height - 1) - grid_y
    px = float(cell.x)
    py = float(grid_msg.height - 1) - float(cell.y)
    return px, py


def render(
    *,
    recording: Path,
    dataset: Path,
    full_audit: Path,
    output: Path,
) -> Path:
    objects = _collect_objects(full_audit)
    questions, robots, frame_objs = _collect_frame_meta(dataset, full_audit, objects)
    frames = {str(q["frame"]) for q in questions}
    _fill_missing_poses(recording, dataset, robots, frames)
    png_bytes, grid_msg = _costmap_png(recording)
    png_b64 = base64.b64encode(png_bytes).decode("ascii")
    width = int(grid_msg.width)
    height = int(grid_msg.height)

    categories = sorted(
        {str(obj["name"]).casefold(): str(obj["name"]) for obj in objects}.values(),
        key=str.casefold,
    )
    cat_color = {
        name.casefold(): _CAT_COLORS[index % len(_CAT_COLORS)]
        for index, name in enumerate(categories)
    }

    obj_marks: list[str] = []
    for index, obj in enumerate(objects, start=1):
        px, py = _world_to_px(float(obj["x"]), float(obj["y"]), grid_msg)
        color = cat_color[str(obj["name"]).casefold()]
        title = html.escape(
            f"{obj['name']}  ({float(obj['x']):.2f}, {float(obj['y']):.2f})"
        )
        obj_marks.append(
            f'<g class="mark obj-g" data-i="{index}">'
            f'<circle class="obj-dot" cx="{px:.1f}" cy="{py:.1f}" r="3.2" fill="{color}" '
            f'stroke="#111" stroke-width="0.6"><title>{title}</title></circle>'
            f'<circle class="obj-ring" cx="{px:.1f}" cy="{py:.1f}" r="5" fill="none" '
            f'stroke="#c00" stroke-width="1.1" opacity="0"/>'
            f'<text class="lab" x="{px + 4:.1f}" y="{py - 4:.1f}">{html.escape(str(index))}</text>'
            f"</g>"
        )

    robot_marks: list[str] = []
    for frame in sorted(frames):
        if frame not in robots:
            continue
        (x, y, _z), yaw = robots[frame]
        px, py = _world_to_px(x, y, grid_msg)
        deg = -math.degrees(yaw)
        idx = frame.split("-")[1].lstrip("0") or "0"
        title = html.escape(f"{frame}  yaw={yaw:.2f}")
        # Short dashed heading + light FOV rays (map px; +x is forward after yaw rotate).
        robot_marks.append(
            f'<g class="mark robot-g" data-frame="{html.escape(frame)}" '
            f'transform="translate({px:.1f},{py:.1f})">'
            f'<g transform="rotate({deg:.1f})">'
            f'<line class="view-ray" x1="4" y1="0" x2="22" y2="0"/>'
            f'<line class="view-fov" x1="4" y1="0" x2="18" y2="-7"/>'
            f'<line class="view-fov" x1="4" y1="0" x2="18" y2="7"/>'
            f'<polygon points="5,0 -3.5,3 -3.5,-3" fill="#111" stroke="#fff" stroke-width="0.5"/>'
            f"</g>"
            f'<text class="lab" x="4" y="12">{html.escape(idx)}</text>'
            f"<title>{title}</title>"
            f"</g>"
        )

    by_frame: dict[str, list[dict[str, object]]] = defaultdict(list)
    for q in questions:
        by_frame[str(q["frame"])].append(q)

    q_rows: list[str] = []
    for frame in sorted(by_frame):
        pose = robots.get(frame)
        pose_s = ""
        if pose is not None:
            (x, y, _z), yaw = pose
            pose_s = f"{x:.2f},{y:.2f} θ={math.degrees(yaw):.0f}°"
        frame_obj_csv = ",".join(str(i) for i in frame_objs.get(frame, []))
        for qi, q in enumerate(by_frame[frame]):
            # Question-specific referents only. Image-path families (presence, …)
            # have no 3D evidence — do not fall back to every localize object on
            # the frame (that made presence clicks light up many dots).
            q_ids = [int(i) for i in (q.get("obj_ids") or ())]
            q_objs = ",".join(str(i) for i in q_ids)
            frame_cell = (
                f'<td class="fr">'
                f'<button type="button" class="frame-btn" data-frame="{html.escape(frame)}" '
                f'data-objs="{html.escape(frame_obj_csv)}">'
                f"{html.escape(frame.split('-')[1])}"
                f"</button>"
                f'<div class="pose">{html.escape(pose_s)} · objs {len(frame_objs.get(frame, []))}</div></td>'
                if qi == 0
                else "<td></td>"
            )
            q_rows.append(
                f'<tr class="qrow" data-frame="{html.escape(frame)}" '
                f'data-objs="{html.escape(q_objs)}" tabindex="0">'
                f"{frame_cell}"
                f'<td class="fam">{html.escape(str(q["family"]))}</td>'
                f'<td class="ans">{html.escape(str(q["answer"]))}</td>'
                f'<td class="qq">{html.escape(str(q["question"]))}</td>'
                f"</tr>"
            )

    legend = "".join(
        f'<span class="leg"><i style="background:{cat_color[name.casefold()]}"></i>'
        f"{html.escape(name)}</span>"
        for name in categories
    )
    obj_list = "".join(
        f'<div class="obj-item" data-i="{index}"><b>{index}</b> '
        f"{html.escape(str(obj['name']))} "
        f"({float(obj['x']):.1f},{float(obj['y']):.1f})</div>"
        for index, obj in enumerate(objects, start=1)
    )

    page = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>VQA topdown audit — {html.escape(dataset.name)}</title>
<style>
  :root {{
    --bg: #f7f7f5;
    --ink: #151515;
    --muted: #666;
    --line: #ddd;
    --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    --sans: system-ui, -apple-system, Segoe UI, sans-serif;
  }}
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0;
    color: var(--ink);
    background: var(--bg);
    font: 12px/1.35 var(--sans);
  }}
  header {{
    display: flex;
    flex-wrap: wrap;
    align-items: flex-start;
    justify-content: space-between;
    gap: 8px 16px;
    padding: 10px 12px;
    border-bottom: 1px solid var(--line);
    background: #fff;
  }}
  header h1 {{
    margin: 0 0 4px;
    font-size: 14px;
    font-weight: 650;
  }}
  header p {{ margin: 0; color: var(--muted); }}
  .hdr-actions {{ display: flex; gap: 8px; align-items: center; }}
  button.show-all {{
    border: 1px solid #bbb;
    background: #fff;
    padding: 5px 10px;
    font: 600 12px var(--sans);
    cursor: pointer;
  }}
  button.show-all:hover,
  button.show-all:focus-visible {{ border-color: #111; outline: none; }}
  .wrap {{
    display: grid;
    grid-template-columns: minmax(0, 1.15fr) minmax(320px, 0.85fr);
    gap: 0;
    min-height: calc(100vh - 54px);
  }}
  @media (max-width: 980px) {{
    .wrap {{ grid-template-columns: 1fr; }}
  }}
  .map-pane {{
    padding: 10px 12px;
    border-right: 1px solid var(--line);
    background: #fff;
  }}
  .map-box {{
    position: relative;
    width: 100%;
    max-width: 920px;
    border: 1px solid #bbb;
    background: #fff;
    line-height: 0;
  }}
  .map-box img, .map-box svg {{
    width: 100%;
    height: auto;
    display: block;
  }}
  .map-box svg {{
    position: absolute;
    inset: 0;
  }}
  .lab {{
    font: 500 7px var(--mono);
    fill: #111;
    paint-order: stroke;
    stroke: #fff;
    stroke-width: 1.5px;
  }}
  .robot-g {{ cursor: pointer; }}
  .view-ray {{
    stroke: #111;
    stroke-width: 1.1;
    stroke-dasharray: 2.5 1.8;
    stroke-linecap: round;
    fill: none;
  }}
  .view-fov {{
    stroke: #666;
    stroke-width: 0.7;
    stroke-dasharray: 2 2;
    stroke-linecap: round;
    fill: none;
    opacity: 0.7;
  }}
  body.is-focus .robot-g.is-hot .view-ray,
  body.is-focus .robot-g.is-hot .view-fov {{ stroke: #c00; opacity: 1; }}
  .mark {{ transition: opacity 120ms linear; }}
  body.is-focus .mark {{ opacity: 0.07; }}
  body.is-focus .mark.is-hot {{ opacity: 1; }}
  body.is-focus .robot-g.is-hot polygon {{ fill: #c00; }}
  body.is-focus .obj-g.is-hot .obj-ring {{ opacity: 1; }}
  .legends {{
    display: flex;
    flex-wrap: wrap;
    gap: 6px 10px;
    margin-top: 8px;
  }}
  .leg {{
    display: inline-flex;
    align-items: center;
    gap: 4px;
    color: var(--muted);
    font-size: 11px;
  }}
  .leg i {{
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
    border: 1px solid #333;
  }}
  .side {{
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    min-height: 0;
    height: calc(100vh - 54px);
    max-height: calc(100vh - 54px);
    overflow: hidden;
  }}
  .objs {{
    max-height: 160px;
    overflow: auto;
    padding: 8px 10px;
    border-bottom: 1px solid var(--line);
    background: #fff;
    font: 11px/1.3 var(--mono);
    columns: 2;
    column-gap: 12px;
  }}
  .objs b {{ color: var(--muted); font-weight: 600; }}
  body.is-focus .obj-item {{ opacity: 0.22; }}
  body.is-focus .obj-item.is-hot {{ opacity: 1; font-weight: 650; }}
  .qs {{
    min-height: 0;
    overflow-x: hidden;
    overflow-y: scroll;
    background: #fff;
    scrollbar-gutter: stable;
  }}
  .qs::-webkit-scrollbar {{ width: 10px; }}
  .qs::-webkit-scrollbar-thumb {{
    background: #bbb;
    border-radius: 5px;
  }}
  .qs::-webkit-scrollbar-track {{ background: #eee; }}
  table {{
    width: 100%;
    border-collapse: collapse;
    font: 11px/1.3 var(--mono);
  }}
  th, td {{
    padding: 3px 6px;
    border-bottom: 1px solid var(--line);
    vertical-align: top;
    text-align: left;
  }}
  th {{
    position: sticky;
    top: 0;
    background: #f0f0ee;
    font: 600 11px var(--sans);
  }}
  tr.qrow {{ cursor: pointer; }}
  tr.qrow:hover {{ background: #f5f5f2; }}
  tr.qrow:focus-visible {{ outline: 1px solid #111; outline-offset: -1px; }}
  body.is-focus tr.qrow {{ opacity: 0.28; }}
  body.is-focus tr.qrow.is-hot {{
    opacity: 1;
    background: #fff3bf;
  }}
  .fr {{ width: 72px; white-space: nowrap; }}
  .frame-btn {{
    margin: 0;
    padding: 0;
    border: 0;
    background: none;
    color: #0b57d0;
    font: 650 11px var(--mono);
    text-decoration: underline;
    cursor: pointer;
  }}
  .frame-btn:hover,
  .frame-btn:focus-visible {{ color: #111; outline: none; }}
  .pose {{ color: var(--muted); font-size: 10px; }}
  .fam {{ width: 110px; color: var(--muted); }}
  .ans {{ width: 72px; font-weight: 650; }}
  .qq {{ overflow-wrap: anywhere; }}
  .note {{ margin-top: 6px; color: var(--muted); font-size: 11px; }}
</style>
</head>
<body>
<header>
  <div>
    <h1>Top-down VQA audit — {html.escape(dataset.name)}</h1>
    <p>
      costmap {width}×{height} @ {float(grid_msg.resolution):.3f}m ·
      objects {len(objects)} · frames {len(frames)} · questions {len(questions)} ·
      robots drawn {sum(1 for f in frames if f in robots)}
    </p>
  </div>
  <div class="hdr-actions">
    <button type="button" class="show-all" id="show-all">Show all</button>
  </div>
</header>
<div class="wrap">
  <section class="map-pane">
    <div class="map-box">
      <img width="{width}" height="{height}" alt="global costmap top-down"
           src="data:image/png;base64,{png_b64}">
      <svg viewBox="0 0 {width} {height}" preserveAspectRatio="xMidYMid meet"
           role="img" aria-label="objects and robot poses">
        {"".join(obj_marks)}
        {"".join(robot_marks)}
      </svg>
    </div>
    <div class="legends">{legend}
      <span class="leg"><i style="background:#111;border-radius:1px"></i>robot + dashed view</span>
    </div>
    <p class="note">
      Click frame id / robot: highlight that pose + all localize objects from the frame audit.
      Click a localize question for its specific referent (or matching category for
      any-ahead). Image-path rows (presence, count, …) highlight the pose only.
      Show all resets.
    </p>
  </section>
  <section class="side">
    <div class="objs">{obj_list}</div>
    <div class="qs" id="qs">
      <table>
        <thead><tr><th>frame</th><th>family</th><th>gt</th><th>question</th></tr></thead>
        <tbody>
          {"".join(q_rows)}
        </tbody>
      </table>
    </div>
  </section>
</div>
<script>
(() => {{
  const body = document.body;
  const showAllBtn = document.getElementById("show-all");
  const robots = [...document.querySelectorAll(".robot-g")];
  const objs = [...document.querySelectorAll(".obj-g")];
  const objItems = [...document.querySelectorAll(".obj-item")];
  const rows = [...document.querySelectorAll("tr.qrow")];
  const frameBtns = [...document.querySelectorAll(".frame-btn")];
  const qsPane = document.getElementById("qs");

  const parseObjs = (value) =>
    (value || "")
      .split(",")
      .map((s) => s.trim())
      .filter(Boolean);

  const clear = () => {{
    body.classList.remove("is-focus");
    robots.forEach((el) => el.classList.remove("is-hot"));
    objs.forEach((el) => el.classList.remove("is-hot"));
    objItems.forEach((el) => el.classList.remove("is-hot"));
    rows.forEach((el) => el.classList.remove("is-hot"));
  }};

  const focus = ({{ frame, objIds, scrollRow }}) => {{
    clear();
    body.classList.add("is-focus");
    const idSet = new Set(objIds);
    robots.forEach((el) => {{
      if (el.getAttribute("data-frame") === frame) el.classList.add("is-hot");
    }});
    objs.forEach((el) => {{
      if (idSet.has(el.getAttribute("data-i"))) el.classList.add("is-hot");
    }});
    objItems.forEach((el) => {{
      if (idSet.has(el.getAttribute("data-i"))) el.classList.add("is-hot");
    }});
    const frameRows = rows.filter((el) => el.getAttribute("data-frame") === frame);
    frameRows.forEach((el) => el.classList.add("is-hot"));
    const target = scrollRow || frameRows[0];
    if (target && qsPane) {{
      const top = target.offsetTop - qsPane.clientHeight * 0.25;
      qsPane.scrollTo({{ top: Math.max(0, top), behavior: "smooth" }});
    }}
  }};

  showAllBtn?.addEventListener("click", clear);

  rows.forEach((row) => {{
    const activate = () => {{
      focus({{
        frame: row.getAttribute("data-frame"),
        objIds: parseObjs(row.getAttribute("data-objs")),
        scrollRow: row,
      }});
    }};
    row.addEventListener("click", (event) => {{
      if (event.target.closest(".frame-btn")) return;
      activate();
    }});
    row.addEventListener("keydown", (event) => {{
      if (event.key === "Enter" || event.key === " ") {{
        event.preventDefault();
        activate();
      }}
    }});
  }});

  frameBtns.forEach((btn) => {{
    btn.addEventListener("click", (event) => {{
      event.stopPropagation();
      focus({{
        frame: btn.getAttribute("data-frame"),
        objIds: parseObjs(btn.getAttribute("data-objs")),
      }});
    }});
  }});

  robots.forEach((robot) => {{
    robot.addEventListener("click", () => {{
      const frame = robot.getAttribute("data-frame");
      const frameRows = rows.filter((el) => el.getAttribute("data-frame") === frame);
      const btn = frameBtns.find((el) => el.getAttribute("data-frame") === frame);
      focus({{
        frame,
        objIds: parseObjs(btn?.getAttribute("data-objs")),
        scrollRow: frameRows[0],
      }});
    }});
  }});
}})();
</script>
</body>
</html>
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(page, encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--recording",
        type=Path,
        default=Path("data/aa5f2481ad2a-recording_go2.db"),
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("data/vqa_aa5f_localize_full_k5"),
    )
    parser.add_argument(
        "--full-audit",
        type=Path,
        default=None,
        help="Audit tree with localize evidence (defaults to sibling *_full/audit)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    full_audit = args.full_audit
    if full_audit is None:
        sibling = args.dataset.parent / "vqa_aa5f_localize_full" / "audit"
        full_audit = sibling if sibling.exists() else args.dataset / "audit"
    output = args.output or (args.dataset / "audit_topdown.html")
    path = render(
        recording=args.recording,
        dataset=args.dataset,
        full_audit=full_audit,
        output=output,
    )
    print(path)


if __name__ == "__main__":
    main()
