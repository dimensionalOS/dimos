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

"""Check YOLO on the actual scene's native head cameras, outside the running world."""

import json
import time
from copy import copy, deepcopy

import cv2
import mujoco
import numpy as np
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.robot.pollen.microduck.policies import PolicyBank
from microduck_world.camera import HEAD_CAMERA
from microduck_world.football import BALL_NAMES
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import ROBOT_IDS
from microduck_world.roster import SETTINGS
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule


def box_iou(a, b) -> float:
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union else 0.0


def main() -> None:
    output = PROJECT_ROOT / "state/validation/ball-perception"
    output.mkdir(parents=True, exist_ok=True)
    module = WorldSimModule(
        scene_xml=load_world()[0].mujoco_scene_path,
        robot_mjcf=str(PROJECT_ROOT / "assets/microduck/robot/robot_allcollisions.xml"),
        headless=True,
        auto_stand=False,
    )
    source = module._compose_model()
    model = source
    bank = PolicyBank(PROJECT_ROOT / "assets/microduck/policies", model)
    detector = Yolo2DDetector(model_path=str(PROJECT_ROOT / "cache/models"))
    renderer = mujoco.Renderer(model, height=360, width=640)
    option = mujoco.MjvOption()
    option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    records = []
    try:
        for robot in ROBOT_IDS:
            model = copy(source)
            data = mujoco.MjData(model)
            settings = deepcopy(SETTINGS)
            settings["robots"][robot].update(spawn=[0, 5], yaw=0)
            robots = WorldRobots(model, bank, settings, module.config)
            for _ in range(400):
                robots.leases({robot: "camera-validation"})
                robots.step(data)
                mujoco.mj_step(model, data)
            prefix = "" if robot == "duck1" else robot + "_"
            camera = model.camera(prefix + HEAD_CAMERA).id
            root = data.xpos[model.body(prefix + "trunk_base").id].copy()
            for ball in (*BALL_NAMES, "none"):
                for distance in (0.3, 0.5, 0.8, 1.2) if ball != "none" else (0.5,):
                    for other in BALL_NAMES:
                        joint = model.body(other).jntadr[0]
                        adr = model.jnt_qposadr[joint]
                        data.qpos[adr : adr + 3] = (
                            [root[0] + distance, root[1], 0.052] if other == ball else [15, 15, -5]
                        )
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=camera, scene_option=option)
                    rgb = renderer.render().copy()
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    started = time.perf_counter()
                    prediction = detector.model.predict(
                        bgr,
                        device=detector.device,
                        classes=[32],
                        conf=0.25,
                        iou=0.6,
                        imgsz=640,
                        verbose=False,
                    )[0]
                    elapsed = (time.perf_counter() - started) * 1000
                    boxes = prediction.boxes.xyxy.cpu().tolist()
                    scores = prediction.boxes.conf.cpu().tolist()
                    renderer.enable_segmentation_rendering()
                    segmentation = renderer.render().copy()
                    renderer.disable_segmentation_rendering()
                    ground_truth = None
                    if ball != "none":
                        geoms = np.where(model.geom_bodyid == model.body(ball).id)[0]
                        mask = np.isin(segmentation[:, :, 0], geoms) & (
                            segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
                        )
                        ys, xs = np.where(mask)
                        if len(xs):
                            ground_truth = [
                                int(xs.min()),
                                int(ys.min()),
                                int(xs.max() + 1),
                                int(ys.max() + 1),
                            ]
                    filename = f"{robot}-{ball}-{distance:.1f}.jpg"
                    cv2.imwrite(str(output / filename), bgr)
                    records.append(
                        {
                            "robot": robot,
                            "ball": ball,
                            "distance": distance,
                            "truth": ground_truth,
                            "boxes": boxes,
                            "confidence": scores,
                            "best_iou": max((box_iou(ground_truth, b) for b in boxes), default=0)
                            if ground_truth
                            else None,
                            "ms": elapsed,
                            "image": filename,
                        }
                    )
    finally:
        renderer.close()
        detector.stop()
    visible = [r for r in records if r["truth"]]
    empty = [r for r in records if r["ball"] == "none"]
    report = {
        "mujoco": mujoco.__version__,
        "device": detector.device,
        "model": "yolo11n.pt",
        "threshold": 0.25,
        "frames": len(records),
        "visible_ball_frames": len(visible),
        "detected_visible_frames": sum(bool(r["boxes"]) for r in visible),
        "matched_visible_frames_iou50": sum(r["best_iou"] >= 0.5 for r in visible),
        "empty_false_positive_frames": sum(bool(r["boxes"]) for r in empty),
        "inference_ms_p50": float(np.median([r["ms"] for r in records[1:]])),
        "inference_ms_p95": float(np.percentile([r["ms"] for r in records[1:]], 95)),
        "records": records,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))


if __name__ == "__main__":
    main()
