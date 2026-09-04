"""Side-by-side re-render: MuJoCo sim (left) + humancli-style terminal (right).

Same events.jsonl as render_video.py; instead of subtitles, the right panel
emulates the humancli TUI: prompts typed out live, a thinking indicator while
the agent works, tool calls, and replies, all timestamped from the recording.
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
OUT = "/Users/tule/Desktop/microduck_demo_sidebyside.mp4"
SCENE = (
    "/Users/tule/trenches/dimos-repo/.claude/worktrees/microduck-sim/"
    "dimos/robot/pollen/microduck/assets/room_scene.xml"
)
W, H, FPS = 1280, 720, 30
SIM_W = 800
TERM_W = W - SIM_W
TYPE_SECS = 2.0
SLOW_PRE, SLOW_POST, FAST = 2.5, 6.0, 4

# --- model (same composition as the sim) --------------------------------
scene = mujoco.MjSpec.from_file(SCENE)
robot = mujoco.MjSpec.from_file(str(assets_fetch.robot_mjcf_path()))
scene.visual.global_.offwidth = SIM_W
scene.visual.global_.offheight = H
frame = scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0])
scene.attach(robot, prefix="", frame=frame)
model = scene.compile()
data = mujoco.MjData(model)
hinge_maps = []
free_adr = None
for j in range(model.njnt):
    if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
        free_adr = model.jnt_qposadr[j]
for m in build_joint_mappings(None, model):
    if m.qpos_adr is not None:
        hinge_maps.append(m)

# --- events -------------------------------------------------------------
odom, joints, chat = [], [], []
for line in open(EVENTS):
    e = json.loads(line)
    if e["kind"] == "odom":
        odom.append((e["t"], e["pos"], e["quat_xyzw"]))
    elif e["kind"] == "joints":
        joints.append((e["t"], e["pos"]))
    elif e["kind"] == "chat":
        chat.append((e["t"], e["role"], e["text"]))
T_END = max(odom[-1][0], joints[-1][0])
ot = np.array([o[0] for o in odom])
op = np.array([o[1] for o in odom])
oq = np.array([o[2] for o in odom])
jt = np.array([j[0] for j in joints])
jp = np.array([j[1] for j in joints])

slow_windows = [
    (t - SLOW_PRE, t + SLOW_POST) for t, role, _ in chat if role in ("human", "agent")
]


def speed_at(t):
    return 1 if any(a <= t <= b for a, b in slow_windows) else FAST


def sample(t):
    i = min(np.searchsorted(ot, t), len(ot) - 1)
    k = min(np.searchsorted(jt, t), len(jt) - 1)
    return op[i], oq[i], jp[k]


# --- terminal panel -----------------------------------------------------
MONO = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 15)
MONO_B = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 15, index=1)
CW = MONO.getlength("M")
COLS = int((TERM_W - 36) // CW)
LH = 21

C_BG = (14, 14, 18)
C_HDR = (32, 32, 40)
C_YOU = (110, 200, 255)
C_DUCK = (255, 205, 90)
C_DIM = (120, 120, 130)
C_TOOL = (140, 180, 140)
C_TXT = (220, 220, 225)


def clean(s):
    return "".join(ch for ch in s if 31 < ord(ch) < 0x2500)


def wrap_mono(text, width):
    out, cur = [], ""
    for w_ in text.split():
        if len(cur) + len(w_) + 1 > width and cur:
            out.append(cur)
            cur = w_
        else:
            cur = (cur + " " + w_).strip()
    if cur:
        out.append(cur)
    return out or [""]


def stamp(t):
    return f"{int(t) // 60:02d}:{int(t) % 60:02d}"


def term_lines(t):
    """Chat history rendered as (color, bold, text) rows, plus input line."""
    rows = []
    typing = None
    thinking = False
    for ct, role, text in chat:
        if role == "human":
            if ct - TYPE_SECS <= t < ct:
                frac = (t - (ct - TYPE_SECS)) / (TYPE_SECS * 0.85)
                typing = text[: max(0, int(len(text) * min(1.0, frac)))]
                break
            if ct > t:
                break
            first = True
            for ln in wrap_mono(f"[{stamp(ct)}] you > {clean(text)}", COLS):
                rows.append((C_YOU, first, ln))
                first = False
            thinking = True
        elif ct > t:
            break
        elif role == "agent":
            first = True
            for ln in wrap_mono(f"[{stamp(ct)}] duck > {clean(text)}", COLS):
                rows.append((C_DUCK, first, ln))
                first = False
            thinking = False
        elif role == "tool_call":
            for ln in wrap_mono("  -> " + clean(text), COLS):
                rows.append((C_DIM, False, ln))
        elif role == "tool":
            short = clean(text)
            short = short if len(short) <= 90 else short[:87] + "..."
            for ln in wrap_mono("  " + short, COLS):
                rows.append((C_TOOL, False, ln))
        rows.append((C_BG, False, ""))
    if thinking and typing is None:
        dots = "." * (1 + int(t * 2.5) % 3)
        rows.append((C_DIM, False, f"thinking{dots}"))
    return rows, typing


def draw_terminal(t):
    img = Image.new("RGB", (TERM_W, H), C_BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, TERM_W, 40], fill=C_HDR)
    for i, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
        d.ellipse([14 + i * 22, 14, 26 + i * 22, 26], fill=c)
    d.text((84, 11), "microduck - humancli", font=MONO_B, fill=C_TXT)

    rows, typing = term_lines(t)
    input_y = H - 44
    avail = (input_y - 56) // LH
    rows = rows[-avail:]
    y = 56
    for color, bold, text in rows:
        if text:
            d.text((18, y), text, font=MONO_B if bold else MONO, fill=color)
        y += LH
    d.line([12, input_y - 6, TERM_W - 12, input_y - 6], fill=(50, 50, 60))
    cursor = "█" if int(t * 2) % 2 == 0 else " "
    prompt = "> " + (typing if typing is not None else "") + cursor
    for ln in wrap_mono(prompt, COLS)[-1:]:
        d.text((18, input_y + 8), ln, font=MONO, fill=C_TXT)
    return img


# --- render -------------------------------------------------------------
renderer = mujoco.Renderer(model, height=H, width=SIM_W)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 150, -28, 3.0
lookat = np.array([0.0, 0.0, 0.15])
canvas = Image.new("RGB", (W, H))
FONT_BADGE = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 26)

ff = subprocess.Popen(
    ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
     "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
     "-crf", "20", "-pix_fmt", "yuv420p", OUT],
    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
)

t, n = 0.0, 0
while t < T_END:
    spd = speed_at(t)
    pos, quat, jpos = sample(t)
    if free_adr is not None:
        x, y_, z, w_ = quat
        data.qpos[free_adr : free_adr + 3] = pos
        data.qpos[free_adr + 3 : free_adr + 7] = [w_, x, y_, z]
    for i, m in enumerate(hinge_maps[: len(jpos)]):
        data.qpos[m.qpos_adr] = jpos[i]
    mujoco.mj_forward(model, data)
    lookat += 0.06 * (np.array([pos[0], pos[1], 0.15]) - lookat)
    cam.lookat[:] = lookat
    renderer.update_scene(data, camera=cam)
    sim_img = Image.fromarray(renderer.render())
    if spd > 1:
        ImageDraw.Draw(sim_img).text((SIM_W - 70, 20), f"{spd}x", font=FONT_BADGE, fill=(255, 255, 255))
    canvas.paste(sim_img, (0, 0))
    canvas.paste(draw_terminal(t), (SIM_W, 0))
    ff.stdin.write(np.asarray(canvas).tobytes())
    n += 1
    t += spd / FPS

ff.stdin.close()
ff.wait()
print(f"wrote {OUT}: {n} frames = {n / FPS:.1f}s")
