"""Render still images of each keyframe pose. Usage: python -m tars_sdk.model.render [--out renders]"""

import argparse
from pathlib import Path

import mujoco
from PIL import Image

from tars_sdk.model import MJCF_PATH

SHOTS = {
    # name: (keyframe, lookat z, distance, azimuth, elevation)
    "stand_hero": ("stand", 0.8, 3.4, 145, -12),
    "stand_front": ("stand", 0.8, 3.2, 180, -5),
    "stand_side": ("stand", 0.8, 3.2, 90, -5),
    "display_closeup": ("stand", 1.3, 1.3, 160, -10),
    "stride_side": ("stride", 0.8, 3.4, 110, -8),
    "wheel": ("wheel", 1.3, 4.6, 120, -10),
}


def render(out: Path, w=1600, h=1200):
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=h, width=w)
    cam = mujoco.MjvCamera()
    out.mkdir(parents=True, exist_ok=True)
    images = []
    for name, (key, z, dist, az, el) in SHOTS.items():
        mujoco.mj_resetDataKeyframe(
            model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key)
        )
        mujoco.mj_forward(model, data)
        cam.lookat[:] = (data.qpos[0], data.qpos[1], z)
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        renderer.update_scene(data, camera=cam)
        img = Image.fromarray(renderer.render())
        img.save(out / f"{name}.png")
        images.append(img)
    # contact sheet
    tw, th = w // 2, h // 2
    sheet = Image.new("RGB", (tw * 3, th * 2))
    for k, img in enumerate(images):
        sheet.paste(img.resize((tw, th)), ((k % 3) * tw, (k // 3) * th))
    sheet.save(out / "sheet.png")
    print(f"wrote {len(images)} renders + sheet to {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="renders")
    render(Path(ap.parse_args().out))
