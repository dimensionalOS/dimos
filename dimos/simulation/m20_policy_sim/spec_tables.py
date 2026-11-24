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

"""Regenerate SPEC.md's per-runner tables from m20_controller.toml (the section between the GENERATED markers).

python -m dimos.simulation.m20_policy_sim.spec_tables
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.simulation.m20_policy_sim.config import JOINT_SETS, load_config

SPEC = Path(__file__).with_name("SPEC.md")
BEGIN, END = "<!-- GENERATED: spec_tables.py -->", "<!-- END GENERATED -->"
JOINT = [
    f"{leg}_{part}"
    for leg in ("fl", "fr", "hl", "hr")
    for part in ("hipx", "hipy", "knee", "wheel")
]
BLOCK = {
    "gyro": ("body angular velocity x, y, z", "rad/s"),
    "gravity": ("projected gravity R^T (0, 0, -1) x, y, z", "unit"),
    "accel": ("accelerometer specific force x, y, z (+1 g on z at rest)", "m/s^2"),
    "command": ("velocity command vx, vy, wz (after max / slew)", "m/s, m/s, rad/s"),
    "phase": ("gait clock sin(2 pi phi), cos(2 pi phi)", "-"),
    "last_action": ("previous clipped network output (network output order)", "-"),
}


def fmt(v: float) -> str:
    return f"{v:.6g}"


def scale_text(block: dict[str, Any], k: int) -> str:
    scale = block.get("scale")
    if not scale:
        return "1"
    return fmt(scale[k % len(scale)])


def obs_rows(spec: dict[str, Any]) -> list[str]:
    rows, i = [], 0
    for b in spec["obs"]:
        kind = b["kind"]
        if kind in BLOCK:
            n = 16 if kind == "last_action" else 2 if kind == "phase" else 3
            meaning, unit = BLOCK[kind]
            extra = f", phi += {fmt(b['frequency'])} x 0.02 per step" if kind == "phase" else ""
            scales = sorted({scale_text(b, k) for k in range(n)}, key=str)
            rows.append(
                f"| {i}-{i + n - 1} | {meaning}{extra} | {unit} | {', '.join(scales) if kind != 'command' else ', '.join(scale_text(b, k) for k in range(3))} |"
            )
            i += n
        elif kind == "constant":
            vals = ", ".join(fmt(v) for v in b["values"])
            name = f" (param `{b['name']}`)" if "name" in b else ""
            rows.append(f"| {i}-{i + len(b['values']) - 1} | constant {vals}{name} | - | 1 |")
            i += len(b["values"])
        elif kind in ("joint_pos", "joint_vel"):
            idx = JOINT_SETS[b["joints"]]
            names = ", ".join(JOINT[j] for j in idx)
            what = (
                "joint position"
                + (
                    " - default"
                    if kind == "joint_pos" and b.get("relative", True)
                    else " (absolute)"
                )
                if kind == "joint_pos"
                else "joint velocity"
            )
            scales = ", ".join(scale_text(b, k) for k in range(min(len(b.get("scale") or [1]), 4)))
            rows.append(
                f"| {i}-{i + len(idx) - 1} | {what}: {names} | {'rad' if kind == 'joint_pos' else 'rad/s'} | {scales} |"
            )
            i += len(idx)
        elif kind == "height_map":
            n = b["rows"] * b["cols"]
            rows.append(
                f"| {i}-{i + n - 1} | height map {b['rows']} x {b['cols']}, row-major (see HeightGap notes) | m | {scale_text(b, 0)} |"
            )
            i += n
    return rows


def command_rows(spec: dict[str, Any]) -> list[str]:
    rows = []
    for k, e in enumerate(spec["command_vector"]):
        if not isinstance(e, dict):
            rows.append(f"| {k} | constant {fmt(e)} |")
            continue
        src = e["src"]
        if src in ("vx", "vy", "wz"):
            text = f"command {src}"
        elif src == "param" and "scale_deg" in e:
            text = f"param `{e['name']}` x {fmt(e['scale_deg'])} deg (rad)"
        elif src == "param":
            text = f"param `{e['name']}` x {fmt(e.get('scale', 1.0))}" + (
                f" + {fmt(e['offset'])}" if e.get("offset") else ""
            )
        elif src == "stand_mode":
            text = (
                "stand mode: -mode_byte, 0 while armed and upright (100 steps, re-armed on 0xf003)"
            )
        elif src == "gait":
            text = f"gait clock {e['field']} x {fmt(e.get('scale', 1.0))}"
        else:
            text = f"foot {('FL', 'FR', 'HL', 'HR')[e['leg']]} {e['axis']}: Rz(yaw_deg x yaw) nominal + (dx_scale dx, dy_scale dy), or (nominal x, fixed_y) with feet_fixed"
        rows.append(f"| {k} | {text} |")
    return rows


def runner_section(name: str, spec: dict[str, Any], cfg: dict[str, Any]) -> str:
    gaits = ", ".join(f"{g:#06x}" for g in spec["gaits"]) or "none"
    act = spec["action"]
    kp = ", ".join(fmt(v) for v in spec["kp"])
    kd = ", ".join(fmt(v) for v in spec["kd"])
    pose = spec["default_pose"]
    front, hind = pose[0:3], pose[8:11]
    hist = (
        f"network input `state` [1, {spec['frame'] * (spec['history'] + 1)}] = [current frame, {spec['history']} past frames oldest -> newest] (the history is updated first, so the current frame appears twice)"
        if spec["kind"] == "frames"
        else f"network inputs `state` [1, {spec['history']}, {spec['frame']}] (oldest first, current last), `command` [1, {len(spec['command_vector'])}], `uncertain` [1, 1] = 0"
    )
    lines = [
        f"### {name}: `{spec['policy']}` (gaits {gaits})",
        "",
        f"- {hist}; outputs `output` [1, 16]"
        + (", `vel` [1, 3] (base velocity estimate)" if spec["kind"] == "frames" else ""),
        f"- default pose FL (hipx, hipy, knee) = ({', '.join(fmt(v) for v in front)}), HL = ({', '.join(fmt(v) for v in hind)}); FR / HR mirror hipx"
        + ("" if abs(pose[4] + pose[0]) < 1e-9 or pose[0] == 0 else " (same sign)"),
        f"- kp ({kp}), kd ({kd}) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)"
        + (
            f"; kp / kd x{fmt(spec['start_gain_scale'])} for the first {spec['start_gain_steps']} steps"
            if "start_gain_scale" in spec
            else ""
        ),
        f"- command maxima (vx, vy, wz) = ({', '.join(fmt(v) for v in spec['command_max'])})"
        + (
            f", slew per step ({', '.join(fmt(v) for v in spec['command_slew'])})"
            if "command_slew" in spec
            else ", no slew"
        )
        + (
            f", |wz| <= {fmt(spec['wz_clamp'])} while vx > {fmt(spec['wz_clamp_above_vx'])}"
            if "wz_clamp" in spec
            else ""
        )
        + (f", axis deadband {spec['axis_deadband']}" if "axis_deadband" in spec else "")
        + (", wz max from the speed (see wz_speed_limit)" if "wz_speed_limit" in spec else ""),
        f"- action: {'16 interleaved per leg (hipx, hipy, knee, wheel) x 4' if act['layout'] == 'per_leg' else '12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels'}; "
        + (f"clip +-{fmt(act['clip'])}; " if act["clip"] < 1e8 else "no clip; ")
        + (
            "leg target = default + " + fmt(act["leg_scale"][0]) + " a"
            if act["leg_mode"] == "offset"
            else "leg target = output (the ONNX applies 0.25 and the joint clip itself)"
        )
        + f"; wheel velocity target = {fmt(act['wheel_scale'])} a (rad/s); then rl_deploy's limit_goal_pos_vel clamp (policy_limit_lower / upper, hipx widened by {fmt(cfg['robot']['policy_hipx_widen'])} of its width per end)",
        "",
        f"Frame ({spec['frame']} values):",
        "",
        "| index | content | unit | scale |",
        "|---|---|---|---|",
        *obs_rows(spec),
    ]
    if spec["kind"] == "sequence":
        lines += [
            "",
            "`command` vector:",
            "",
            "| index | value |",
            "|---|---|",
            *command_rows(spec),
        ]
        if "feet" in spec:
            f = spec["feet"]
            lines += [
                "",
                f"Feet: nominal (x, y) FL, FR, HL, HR = {f['nominal']} m, yaw_deg {fmt(f['yaw_deg'])} "
                f"(fixed_y {f['fixed_y']}), dx_scale {fmt(f['dx_scale'])} m, dy_scale {fmt(f['dy_scale'])} m. "
                f"Params (operator inputs): {', '.join(f'{k} = {fmt(v)}' for k, v in spec['command_params'].items())}.",
            ]
        if "wz_speed_limit" in spec:
            w = spec["wz_speed_limit"]
            lines += [
                "",
                f"wz max = clamp(min({fmt(w['a'])} / (vx + {fmt(w['b'])}) - {fmt(w['c'])}, {fmt(w['d'])} / (1 + {fmt(w['e'])} |vy|)), {fmt(w['lo'])}, {fmt(w['hi'])}); wz = axis x wz max.",
            ]
    return "\n".join(lines) + "\n"


def generate() -> str:
    cfg = load_config()
    return "\n".join(runner_section(n, s, cfg) for n, s in cfg["runners"].items())


def main() -> None:
    text = SPEC.read_text()
    a, b = text.index(BEGIN) + len(BEGIN), text.index(END)
    SPEC.write_text(text[:a] + "\n\n" + generate() + "\n" + text[b:])
    print(f"updated {SPEC}")


if __name__ == "__main__":
    main()
