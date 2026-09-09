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

"""Accuracy harness for identify.py.

Builds a synthetic test set that looks like what the Go2 actually sees — a
1280x720 wide-angle frame with a face somewhere in it at 1 m, 2 m or 3 m — then
runs identify.py over it and prints a confusion summary and a threshold sweep.

    uv run python dimos/experimental/frank/tools/test_identify.py --build      # make frames
    uv run python dimos/experimental/frank/tools/test_identify.py              # run the set
    uv run python dimos/experimental/frank/tools/test_identify.py --sweep      # + threshold sweep

Source photos: any directory of `<Person Name>/<photo>.jpg` folders, given with
--source (default: an LFW extract in the scratchpad).  Nothing under app/data/
is committed, and no photo of a real person is written into the repo.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import random
import shutil
import sys
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TEST_DIR = HERE.parent / "app" / "data" / "test"
FRAMES = TEST_DIR / "frames"
GALLERY = TEST_DIR / "gallery"  # acts as FRANK_DATA_DIR for the test run

DEFAULT_SOURCE = Path(
    "/tmp/claude-1000/-home-dimos-dimos/"
    "7a749403-8332-410b-b0eb-36a5b4e7b35f/scratchpad/idf/lfw_funneled"
)

FRAME_W, FRAME_H = 1280, 720

# Face width in pixels at a given distance, for a 1280-wide ~120 deg lens and a
# 16 cm head: 2*atan(0.08/d) / 120deg * 1280.
DISTANCES = {"1m": 100, "2m": 50, "3m": 33}

# In an LFW funneled 250x250 crop the face itself is about this wide.
LFW_FACE_W = 100

N_ENROLLED = 6
N_IMPOSTORS = 4
N_PROBES = 5


def _paste(canvas: np.ndarray, crop: np.ndarray, face_px: int, rng, cx=None) -> None:
    scale = face_px / LFW_FACE_W
    h, w = crop.shape[:2]
    small = cv2.resize(
        crop, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_AREA
    )
    sh, sw = small.shape[:2]
    x = rng.randint(0, FRAME_W - sw) if cx is None else max(0, min(FRAME_W - sw, cx - sw // 2))
    y = rng.randint(0, max(0, FRAME_H - sh))
    canvas[y : y + sh, x : x + sw] = small


def _canvas(rng) -> np.ndarray:
    g = rng.randint(90, 150)
    c = np.full((FRAME_H, FRAME_W, 3), g, dtype=np.uint8)
    c += rng.randint(0, 8)
    return c


def build(source: Path, seed: int = 7) -> None:
    if not source.exists():
        sys.exit(f"no source photos at {source} (pass --source)")
    rng = random.Random(seed)
    people = sorted(p for p in source.iterdir() if p.is_dir())
    if len(people) < N_ENROLLED + N_IMPOSTORS:
        sys.exit(f"need >= {N_ENROLLED + N_IMPOSTORS} identities, found {len(people)}")
    rng.shuffle(people)
    enrolled = people[:N_ENROLLED]
    impostors = people[N_ENROLLED : N_ENROLLED + N_IMPOSTORS]

    for d in (FRAMES, GALLERY / "people"):
        shutil.rmtree(d, ignore_errors=True)
        d.mkdir(parents=True, exist_ok=True)

    manifest = []
    names = {}

    def shots(person: Path) -> list[Path]:
        return sorted(person.glob("*.jpg"))

    for person in enrolled:
        pid = "p_" + person.name.lower().replace("_", "")[:10]
        names[pid] = person.name.replace("_", " ")
        imgs = shots(person)
        # first photo is the enrollment selfie; it never appears as a probe
        shutil.copy(imgs[0], GALLERY / "people" / f"{pid}.jpg")
        for probe in imgs[1 : 1 + N_PROBES]:
            crop = cv2.imread(str(probe))
            for label, px in DISTANCES.items():
                out = FRAMES / f"{pid}__{label}__{probe.stem}.jpg"
                canvas = _canvas(rng)
                _paste(canvas, crop, px, rng)
                cv2.imwrite(str(out), canvas)
                manifest.append({"frame": out.name, "truth": [pid], "distance": label})

    for person in impostors:
        for probe in shots(person)[:N_PROBES]:
            crop = cv2.imread(str(probe))
            out = FRAMES / f"impostor_{person.name}__1m__{probe.stem}.jpg"
            canvas = _canvas(rng)
            _paste(canvas, crop, DISTANCES["1m"], rng)
            cv2.imwrite(str(out), canvas)
            manifest.append({"frame": out.name, "truth": [], "distance": "1m"})

    # in-plane rotated heads: FRANK looks up from the floor, people lean in
    for person in enrolled:
        pid = "p_" + person.name.lower().replace("_", "")[:10]
        for probe in shots(person)[1 : 1 + 2]:
            crop = cv2.imread(str(probe))
            h, w = crop.shape[:2]
            for ang in (25, -25):
                m = cv2.getRotationMatrix2D((w / 2, h / 2), ang, 1.0)
                rot = cv2.warpAffine(crop, m, (w, h), borderMode=cv2.BORDER_REPLICATE)
                out = FRAMES / f"{pid}__tilt{ang}__{probe.stem}.jpg"
                canvas = _canvas(rng)
                _paste(canvas, rot, DISTANCES["1m"], rng)
                cv2.imwrite(str(out), canvas)
                manifest.append({"frame": out.name, "truth": [pid], "distance": "tilt"})

    # two people in one frame
    for i in range(4):
        a, b = enrolled[i % N_ENROLLED], enrolled[(i + 1) % N_ENROLLED]
        pa = "p_" + a.name.lower().replace("_", "")[:10]
        pb = "p_" + b.name.lower().replace("_", "")[:10]
        canvas = _canvas(rng)
        _paste(canvas, cv2.imread(str(shots(a)[1])), 100, rng, cx=320)
        _paste(canvas, cv2.imread(str(shots(b)[1])), 100, rng, cx=960)
        out = FRAMES / f"pair_{i}.jpg"
        cv2.imwrite(str(out), canvas)
        manifest.append({"frame": out.name, "truth": [pa, pb], "distance": "1m"})

    # a frame with nobody in it
    cv2.imwrite(str(FRAMES / "empty_0.jpg"), _canvas(rng))
    manifest.append({"frame": "empty_0.jpg", "truth": [], "distance": "n/a"})

    (GALLERY / "names.json").write_text(json.dumps(names, indent=1))
    (TEST_DIR / "manifest.json").write_text(
        json.dumps({"names": names, "items": manifest}, indent=1)
    )
    print(
        f"built {len(manifest)} frames, {N_ENROLLED} enrolled + {N_IMPOSTORS} impostors -> {FRAMES}"
    )


def _identify_module():
    """Import identify.py with the test gallery as its data dir."""
    os.environ["FRANK_DATA_DIR"] = str(GALLERY)
    sys.path.insert(0, str(HERE))
    import identify

    return identify


def run(threshold: float, sweep: bool) -> int:
    man = json.loads((TEST_DIR / "manifest.json").read_text())
    identify = _identify_module()

    report = identify.enroll(force=True)
    if report["failed"]:
        print("enrollment failures:", report["failed"])
    print(f"enrolled {len(identify.load_db()['people'])} people\n")

    rec = identify.Recognizer(allow_download=True)
    db = identify.load_db()

    # one pass, keeping every (truth, best-match, score) so a sweep is free
    rows = []
    t_total, n_frames = 0.0, 0
    for item in man["items"]:
        img = cv2.imread(str(FRAMES / item["frame"]))
        t0 = time.perf_counter()
        faces = identify.identify_image(img, rec=rec, db=db, threshold=0.0)
        t_total += time.perf_counter() - t0
        n_frames += 1
        rows.append((item, faces))

    def score(th: float) -> dict:
        c = Counter()
        by_dist: dict[str, Counter] = {}
        for item, faces in rows:
            truth = set(item["truth"])
            d = by_dist.setdefault(item["distance"], Counter())
            if not truth:
                for f in faces:
                    hit = f.confidence >= th
                    c["impostor_accept" if hit else "impostor_reject"] += 1
                    d["impostor_accept" if hit else "impostor_reject"] += 1
                if not faces and item["distance"] == "n/a":
                    c["empty_ok"] += 1
                continue
            c["faces_expected"] += len(truth)
            d["faces_expected"] += len(truth)
            if len(faces) < len(truth):
                c["missed_detection"] += len(truth) - len(faces)
                d["missed_detection"] += len(truth) - len(faces)
            for f in faces:
                if f.confidence < th:
                    c["rejected"] += 1
                    d["rejected"] += 1
                elif f.person_id in truth:
                    c["correct"] += 1
                    d["correct"] += 1
                else:
                    c["wrong"] += 1
                    d["wrong"] += 1
        return {"all": c, "by_dist": by_dist}

    s = score(threshold)
    c = s["all"]
    print(f"threshold {threshold}")
    print(f"  frames            {n_frames}")
    print(f"  faces expected    {c['faces_expected']}")
    print(f"  correct           {c['correct']}")
    print(f"  wrong name        {c['wrong']}   <- false positives, must be 0")
    print(f"  rejected(unknown) {c['rejected']}")
    print(f"  missed detection  {c['missed_detection']}")
    print(f"  impostor accepted {c['impostor_accept']}   <- must be 0")
    print(f"  impostor rejected {c['impostor_reject']}")
    print(f"  empty frame clean {c['empty_ok']}")
    print("\n  by distance:")
    for d, dc in s["by_dist"].items():
        exp = dc["faces_expected"]
        if exp:
            print(
                f"    {d:>4}  {dc['correct']}/{exp} correct  "
                f"{dc['rejected']} unknown  {dc['wrong']} wrong  "
                f"{dc['missed_detection']} undetected"
            )
        else:
            print(
                f"    {d:>4}  impostors: {dc['impostor_accept']} accepted, "
                f"{dc['impostor_reject']} rejected"
            )

    print(
        f"\n  timing: {t_total / n_frames * 1e3:.0f} ms per 1280x720 frame "
        f"({n_frames} frames, detect+embed, warm model)"
    )

    if sweep:
        print("\n  threshold sweep (correct / wrong / impostor-accept):")
        for th in [0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
            sc = score(th)["all"]
            print(
                f"    {th:.2f}  {sc['correct']:>4} / {sc['wrong']:>3} / {sc['impostor_accept']:>3}"
            )

    return 0 if c["wrong"] == 0 and c["impostor_accept"] == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", action="store_true", help="regenerate the test frames")
    ap.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    ap.add_argument("--threshold", type=float, default=None)
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()

    if a.build or not (TEST_DIR / "manifest.json").exists():
        build(a.source)
        if a.build:
            return 0

    th = a.threshold
    if th is None:
        sys.path.insert(0, str(HERE))
        os.environ.setdefault("FRANK_DATA_DIR", str(GALLERY))
        import identify

        th = identify.THRESHOLD
    return run(th, a.sweep)


if __name__ == "__main__":
    raise SystemExit(main())
