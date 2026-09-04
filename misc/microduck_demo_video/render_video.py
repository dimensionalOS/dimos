"""Offline re-render of the recorded microduck demo into an mp4.

Replays events.jsonl (odom + 14 joint positions + chat) through the same
composed MuJoCo model the sim used, rendering offscreen on the main thread
(macOS-safe), with the conversation overlaid as subtitles. Dialogue plays at
1x; quiet stretches (the walk) at 4x with a speed badge.
"""

import json
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, "/Users/tule/trenches/dimos-repo/.claude/worktrees/microduck-sim")
from dimos.robot.pollen.microduck import assets_fetch  # noqa: E402
from dimos.simulation.utils.xml_parser import build_joint_mappings  # noqa: E402

EVENTS = "/Users/tule/.claude/jobs/1c2d08cf/tmp/events.jsonl"
OUT = "/Users/tule/Desktop/microduck_demo.mp4"
SCENE = (
    "/Users/tule/trenches/dimos-repo/.claude/worktrees/microduck-sim/"
    "dimos/robot/pollen/microduck/assets/room_scene.xml"
)
W, H, FPS = 1280, 720, 30
SLOW_PRE, SLOW_POST, FAST = 0.5, 6.0, 4

# --- compose model (mirrors MicroduckSimModule._compose_model, no lidar cams)
scene = mujoco.MjSpec.from_file(SCENE)
robot = mujoco.MjSpec.from_file(str(assets_fetch.robot_mjcf_path()))
scene.option.timestep = 0.005
robot.option.timestep = 0.005
scene.visual.global_.offwidth = W
scene.visual.global_.offheight = H
frame = scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0])
scene.attach(robot, prefix="", frame=frame)
model = scene.compile()
data = mujoco.MjData(model)

mappings = [
    m for m in build_joint_mappings(None, model) if m.qpos_adr is not None
]
free_adr = None
hinge_maps = []
for j in range(model.njnt):
    if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
        free_adr = model.jnt_qposadr[j]
for m in mappings:
    jid = getattr(m, "joint_id", None)
    if jid is not None and model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
        continue
    hinge_maps.append(m)
print(f"model: {model.njnt} joints, free_adr={free_adr}, mapped={len(hinge_maps)}")

# --- load events --------------------------------------------------------
odom, joints, chat = [], [], []
for line in open(EVENTS):
    e = json.loads(line)
    if e["kind"] == "odom":
        odom.append((e["t"], e["pos"], e["quat_xyzw"]))
    elif e["kind"] == "joints":
        joints.append((e["t"], e["pos"]))
    elif e["kind"] == "chat":
        chat.append((e["t"], e["role"], e["text"]))
assert odom and joints, f"missing streams: odom={len(odom)} joints={len(joints)}"
T_END = max(odom[-1][0], joints[-1][0])
print(f"events: {len(odom)} odom, {len(joints)} joints, {len(chat)} chat, T={T_END:.0f}s")

ot = np.array([o[0] for o in odom])
op = np.array([o[1] for o in odom])
oq = np.array([o[2] for o in odom])
jt = np.array([j[0] for j in joints])
jp = np.array([j[1] for j in joints])


def sample(t: float):
    i = min(np.searchsorted(ot, t), len(ot) - 1)
    pos, quat = op[i], oq[i]
    k = min(np.searchsorted(jt, t), len(jt) - 1)
    return pos, quat, jp[k]


# --- time warp: 1x around dialogue, FAST elsewhere ----------------------
slow_windows = [
    (t - SLOW_PRE, t + SLOW_POST) for t, role, _ in chat if role in ("human", "agent")
]


def speed_at(t: float) -> int:
    return 1 if any(a <= t <= b for a, b in slow_windows) else FAST


# --- subtitles ----------------------------------------------------------
def ascii_clean(s: str) -> str:
    return "".join(ch for ch in s if ord(ch) < 0x2500).strip()


font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 26)
font_small = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 20)


def wrap(draw, text, fnt, maxw):
    words, lines, cur = text.split(), [], ""
    for w_ in words:
        trial = (cur + " " + w_).strip()
        if draw.textlength(trial, font=fnt) > maxw and cur:
            lines.append(cur)
            cur = w_
        else:
            cur = trial
    if cur:
        lines.append(cur)
    return lines


def overlay(frame_arr, t: float, spd: int):
    img = Image.fromarray(frame_arr).convert("RGBA")
    draw = ImageDraw.Draw(img)
    # active chat lines: last human + last agent + recent tool_call
    human = agent = tool = None
    for ct, role, text in chat:
        if ct > t:
            break
        if role == "human":
            human, agent, tool = (ct, text), None, None
        elif role == "agent":
            agent = (ct, text)
        elif role == "tool_call" and t - ct < 6:
            tool = (ct, text)
    rows = []
    if human:
        rows += [("You", l) for l in wrap(draw, ascii_clean(human[1]), font, W - 220)]
    if tool:
        rows += [("act", l) for l in wrap(draw, ascii_clean(tool[1]), font_small, W - 220)]
    if agent:
        rows += [("Duck", l) for l in wrap(draw, ascii_clean(agent[1]), font, W - 220)]
    if rows:
        pad, lh = 14, 34
        bh = pad * 2 + lh * len(rows)
        bar = Image.new("RGBA", (W, bh), (10, 10, 14, 200))
        img.paste(bar, (0, H - bh), bar)
        y = H - bh + pad
        for tag, line in rows:
            color = {"You": (120, 200, 255), "Duck": (255, 210, 90), "act": (150, 150, 150)}[tag]
            prefix = {"You": "You:  ", "Duck": "Duck: ", "act": "  > "}[tag]
            fnt = font_small if tag == "act" else font
            draw.text((30, y), prefix + line, font=fnt, fill=color + (255,))
            y += lh
    if spd > 1:
        draw.text((W - 90, 24), f"{spd}x", font=font, fill=(255, 255, 255, 220))
    return np.asarray(img.convert("RGB"))


# --- render loop --------------------------------------------------------
renderer = mujoco.Renderer(model, height=H, width=W)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 150, -28, 3.0
lookat = np.array([0.0, 0.0, 0.15])

ff = subprocess.Popen(
    ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
     "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
     "-crf", "20", "-pix_fmt", "yuv420p", OUT],
    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
)

t, nframes = 0.0, 0
while t < T_END:
    spd = speed_at(t)
    pos, quat, jpos = sample(t)
    if free_adr is not None:
        data.qpos[free_adr : free_adr + 3] = pos
        x, y, z, w = quat
        data.qpos[free_adr + 3 : free_adr + 7] = [w, x, y, z]
    for i, m in enumerate(hinge_maps[: len(jpos)]):
        data.qpos[m.qpos_adr] = jpos[i]
    mujoco.mj_forward(model, data)
    target = np.array([pos[0], pos[1], 0.15])
    lookat += 0.06 * (target - lookat)
    cam.lookat[:] = lookat
    renderer.update_scene(data, camera=cam)
    frame_arr = overlay(renderer.render(), t, spd)
    ff.stdin.write(frame_arr.tobytes())
    nframes += 1
    t += spd / FPS

ff.stdin.close()
ff.wait()
print(f"wrote {OUT}: {nframes} frames = {nframes / FPS:.1f}s of video")
