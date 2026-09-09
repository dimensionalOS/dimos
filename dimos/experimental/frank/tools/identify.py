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

"""FRANK face identification.

Detect faces in a frame and match them against enrolled selfies.

    uv run python dimos/experimental/frank/tools/identify.py enroll
    uv run python dimos/experimental/frank/tools/identify.py who frame.jpg
    uv run python dimos/experimental/frank/tools/identify.py who frame.jpg --json

Detector: YuNet.  Recogniser: SFace (128-d).  Both ship as ONNX models run by
OpenCV's own DNN backend, so the only dependency is opencv-python, which the
venv already has.  Models are downloaded once into app/data/models/.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import urllib.request

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent  # dimos/experimental/frank

# FRANK_DATA_DIR lets the test harness point at a scratch gallery instead of the
# live demo data.  Model weights always live with the skill so they download once.
DATA = Path(os.environ.get("FRANK_DATA_DIR", ROOT / "app" / "data"))
PEOPLE_DIR = DATA / "people"
# Models live in cache/, not app/data/, so the end-of-day data wipe does not
# throw away 39 MB of weights.
MODEL_DIR = Path(os.environ.get("FRANK_MODEL_DIR", ROOT / "cache" / "models"))
EMBED_PATH = DATA / "embeddings.json"

YUNET_URL = (
    "https://media.githubusercontent.com/media/opencv/opencv_zoo/main/"
    "models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
)
SFACE_URL = (
    "https://media.githubusercontent.com/media/opencv/opencv_zoo/main/"
    "models/face_recognition_sface/face_recognition_sface_2021dec.onnx"
)

# Cosine similarity threshold.  See the hand-back note: OpenCV ships SFace with
# a recommended cosine threshold of 0.363; we run hotter because a false
# "hello Alice" is much worse for FRANK than an occasional "hello, who are you?".
THRESHOLD = float(os.environ.get("FRANK_FACE_THRESHOLD", "0.45"))

# Faces smaller than this are reported but flagged: the Go2 camera is wide
# angle and detail below ~60 px of face width is not reliable.
MIN_USABLE_PX = 60

# Below this the detector's own crop is too small to embed at all.
MIN_DETECT_PX = 24


class EnrollError(Exception):
    """Something a chat server can show the user verbatim."""


@dataclass
class Face:
    person_id: str | None
    name: str | None
    confidence: float
    bbox: list[int]
    face_px: int
    best_guess: str | None = None

    def line(self) -> str:
        x1, y1, x2, y2 = self.bbox
        who = self.name if self.name else "unknown"
        return f"{who}  {self.confidence:.2f}  bbox={x1},{y1},{x2},{y2}  size={self.face_px}px"


# --------------------------------------------------------------------------- models


def _download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    print(f"downloading {dest.name} ...", file=sys.stderr)
    with urllib.request.urlopen(url, timeout=120) as r, tmp.open("wb") as f:
        while chunk := r.read(1 << 20):
            f.write(chunk)
    tmp.rename(dest)


def _model_paths(allow_download: bool) -> tuple[Path, Path]:
    yunet, sface = MODEL_DIR / "yunet.onnx", MODEL_DIR / "sface.onnx"
    for path, url in ((yunet, YUNET_URL), (sface, SFACE_URL)):
        if not path.exists():
            if not allow_download:
                raise SystemExit(
                    f"missing model {path}. Run `identify.py enroll` once with "
                    "network access to fetch it."
                )
            _download(url, path)
    return yunet, sface


class Recognizer:
    """Detector + embedder.  Construct once, reuse across frames."""

    def __init__(self, allow_download: bool = False) -> None:
        yunet, sface = _model_paths(allow_download)
        self.detector = cv2.FaceDetectorYN.create(
            str(yunet), "", (320, 320), score_threshold=0.7, nms_threshold=0.3, top_k=50
        )
        self.embedder = cv2.FaceRecognizerSF.create(str(sface), "")

    def detect(self, img: np.ndarray) -> np.ndarray:
        """Return YuNet rows [x, y, w, h, 5 landmarks..., score] sorted by area."""
        h, w = img.shape[:2]
        self.detector.setInputSize((w, h))
        _, faces = self.detector.detect(img)
        if faces is None or len(faces) == 0:
            return np.empty((0, 15), dtype=np.float32)
        faces = faces[np.argsort(-(faces[:, 2] * faces[:, 3]))]
        return faces

    def embed(self, img: np.ndarray, row: np.ndarray) -> np.ndarray:
        aligned = self.embedder.alignCrop(img, row)
        vec = self.embedder.feature(aligned).flatten().astype(np.float32)
        n = float(np.linalg.norm(vec))
        return vec / n if n else vec

    def embed_largest(self, img: np.ndarray) -> np.ndarray:
        """Embed the biggest face in an image.  For enrollment selfies."""
        faces = self.detect(img)
        if len(faces) == 0:
            raise EnrollError(
                "No face found in that photo. Hold the phone at arm's length, "
                "face the camera, and make sure the light is in front of you."
            )
        row = faces[0]
        if int(row[2]) < MIN_DETECT_PX:
            raise EnrollError("The face in that photo is too small. Please retake it closer up.")
        return self.embed(img, row)


# --------------------------------------------------------------------------- store


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def load_db() -> dict:
    if not EMBED_PATH.exists():
        return {"version": 1, "people": {}}
    try:
        db = json.loads(EMBED_PATH.read_text())
    except json.JSONDecodeError:
        return {"version": 1, "people": {}}
    if db.get("version") != 1:
        return {"version": 1, "people": {}}
    return db


def save_db(db: dict) -> None:
    EMBED_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = EMBED_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(db))
    tmp.replace(EMBED_PATH)


def _names_from_server() -> dict[str, str]:
    """Best-effort person_id -> name, from the chat server's sqlite if present.

    Never a network call.  Falls back to the person_id when unavailable.
    """
    names_json = DATA / "names.json"
    if names_json.exists():
        try:
            return {str(k): str(v) for k, v in json.loads(names_json.read_text()).items()}
        except (json.JSONDecodeError, AttributeError):
            pass

    dbfile = DATA / "frank.db"
    if not dbfile.exists():
        return {}
    import sqlite3

    try:
        con = sqlite3.connect(f"file:{dbfile}?mode=ro", uri=True)
        rows = con.execute("select person_id, name from people").fetchall()
        con.close()
        return {str(a): str(b) for a, b in rows}
    except sqlite3.Error:
        return {}


def enroll(force: bool = False) -> dict:
    """(Re)build embeddings for every selfie in app/data/people/.

    Returns a report dict: added / updated / unchanged / failed(person_id -> message).
    """
    PEOPLE_DIR.mkdir(parents=True, exist_ok=True)
    db = load_db()
    people = db["people"]
    names = _names_from_server()

    selfies = sorted(
        p for p in PEOPLE_DIR.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")
    )
    seen: set[str] = set()
    report: dict = {"added": [], "updated": [], "unchanged": [], "failed": {}}

    rec: Recognizer | None = None
    for path in selfies:
        pid = path.stem
        seen.add(pid)
        digest = _sha(path)
        prev = people.get(pid)
        if prev and prev.get("sha") == digest and not force:
            if names.get(pid) and prev.get("name") != names[pid]:
                prev["name"] = names[pid]
            report["unchanged"].append(pid)
            continue
        if rec is None:
            rec = Recognizer(allow_download=True)
        img = cv2.imread(str(path))
        if img is None:
            report["failed"][pid] = "Could not read that image file."
            continue
        try:
            vec = rec.embed_largest(img)
        except EnrollError as e:
            report["failed"][pid] = str(e)
            continue
        people[pid] = {
            "sha": digest,
            "name": names.get(pid, prev.get("name") if prev else None) or pid,
            "embedding": [round(float(v), 6) for v in vec],
            "ts": time.time(),
        }
        report["updated" if prev else "added"].append(pid)

    for pid in list(people):
        if pid not in seen:
            del people[pid]
            report.setdefault("removed", []).append(pid)

    save_db(db)
    return report


# --------------------------------------------------------------------------- who


def _gallery(db: dict) -> tuple[list[str], list[str], np.ndarray]:
    ids = [p for p, v in db["people"].items() if v.get("embedding")]
    if not ids:
        return [], [], np.empty((0, 128), dtype=np.float32)
    names = [db["people"][p].get("name") or p for p in ids]
    mat = np.array([db["people"][p]["embedding"] for p in ids], dtype=np.float32)
    return ids, names, mat


def identify_image(
    img: np.ndarray,
    rec: Recognizer | None = None,
    db: dict | None = None,
    threshold: float = THRESHOLD,
) -> list[Face]:
    """Identify every face in a BGR frame.  Largest face first."""
    rec = rec or Recognizer()
    db = db if db is not None else load_db()
    ids, names, mat = _gallery(db)

    out: list[Face] = []
    for row in rec.detect(img):
        x, y, w, h = (int(v) for v in row[:4])
        bbox = [x, y, x + w, y + h]
        if w < MIN_DETECT_PX or len(ids) == 0:
            out.append(Face(None, None, 0.0, bbox, w))
            continue
        vec = rec.embed(img, row)
        sims = mat @ vec
        i = int(np.argmax(sims))
        score = float(sims[i])
        if score >= threshold:
            out.append(Face(ids[i], names[i], score, bbox, w))
        else:
            out.append(Face(None, None, score, bbox, w, best_guess=names[i]))
    return out


def identify_file(path: str | Path, **kw) -> list[Face]:
    img = cv2.imread(str(path))
    if img is None:
        raise SystemExit(f"cannot read image: {path}")
    return identify_image(img, **kw)


# --------------------------------------------------------------------------- cli


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    e = sub.add_parser("enroll", help="(re)build embeddings for app/data/people/")
    e.add_argument("--force", action="store_true", help="recompute even if unchanged")
    e.add_argument("--json", action="store_true")

    w = sub.add_parser("who", help="identify faces in a frame")
    w.add_argument("frame")
    w.add_argument("--json", action="store_true")
    w.add_argument("--threshold", type=float, default=THRESHOLD)
    w.add_argument("--time", action="store_true", help="print timing to stderr")

    sub.add_parser("list", help="show enrolled people")

    a = ap.parse_args(argv)

    if a.cmd == "enroll":
        report = enroll(force=a.force)
        if a.json:
            print(json.dumps(report, indent=2))
        else:
            for k in ("added", "updated", "unchanged", "removed"):
                if report.get(k):
                    print(f"{k}: {', '.join(report[k])}")
            for pid, msg in report["failed"].items():
                print(f"FAILED {pid}: {msg}", file=sys.stderr)
        return 1 if report["failed"] else 0

    if a.cmd == "list":
        db = load_db()
        for pid, v in sorted(db["people"].items()):
            print(f"{pid}\t{v.get('name')}")
        return 0

    t0 = time.perf_counter()
    rec = Recognizer()
    t_load = time.perf_counter() - t0
    t1 = time.perf_counter()
    faces = identify_file(a.frame, rec=rec, threshold=a.threshold)
    t_infer = time.perf_counter() - t1

    if a.time or os.environ.get("FRANK_TIME"):
        print(
            f"[load {t_load * 1e3:.0f} ms, detect+embed {t_infer * 1e3:.0f} ms, "
            f"{len(faces)} face(s)]",
            file=sys.stderr,
        )

    if a.json:
        print(json.dumps([asdict(f) for f in faces]))
    else:
        for f in faces:
            line = f.line()
            if f.name is None and f.best_guess:
                line += f"  (best guess {f.best_guess})"
            if f.face_px < MIN_USABLE_PX:
                line += "  [too small, get closer]"
            print(line)

    if not faces:
        return 5
    return 0 if any(f.name for f in faces) else 4


if __name__ == "__main__":
    raise SystemExit(main())
