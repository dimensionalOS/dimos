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

"""Two ways of looking at an answer: the pictures it came from, and where it landed.

A detection is only believable if you can see the thing in the frame the detector was
shown, so the evidence sheet draws each box on its own image with its score and range
written on it. The 3D page then puts those boxes in the recording's own geometry --
built from the depth thumbnails the index already stores -- so a box floating in an
aisle is as obvious as one sitting on a shelf.
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace.detect import Detection
from dimos.mapping.hyperspace.ingest import thumbnail_stream_for
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

THREE_JS = "https://cdnjs.cloudflare.com/ajax/libs/three.js/0.160.0/three.min.js"

# Found boxes are warm, refusals are not drawn at all. The scene is grey so that
# anything coloured is an answer.
FOUND_COLOR = (255, 138, 46)


def scene_points(
    store: Any,
    tf: Any,
    world_frame: str,
    *,
    every_frame: int = 5,
    every_point: int = 6,
    max_points: int = 160_000,
) -> NDArray[np.float32]:
    """The recording's geometry, from the depth thumbnails the index already holds.

    Each thumbnail is a small point cloud in its camera's own frame -- the pose is
    applied here rather than baked in at ingest, so a loop closure moves the scene
    instead of leaving it silently wrong.
    """
    name = thumbnail_stream_for("")
    if name not in store.list_streams():
        return np.zeros((0, 3), dtype=np.float32)
    rows = []
    for index, observation in enumerate(store.stream(name, dict).order_by("ts")):
        if index % every_frame:
            continue
        payload = observation.data
        points = np.asarray(payload["points_mm"], dtype=np.float32)[::every_point] / 1000.0
        if not len(points):
            continue
        rows.append((str(payload["camera_frame"]), float(payload["ts"]), points))
    if not rows:
        return np.zeros((0, 3), dtype=np.float32)

    poses, valid = tf.batch_get(
        world_frame, [frame for frame, _, _ in rows], [ts for _, ts, _ in rows]
    )
    placed = []
    for (_, _, points), pose, ok in zip(rows, poses, valid, strict=True):
        if not ok:
            continue
        placed.append(points @ pose[:3, :3].T + pose[:3, 3])
    if not placed:
        return np.zeros((0, 3), dtype=np.float32)
    cloud = np.concatenate(placed).astype(np.float32)
    if len(cloud) > max_points:
        cloud = cloud[:: int(np.ceil(len(cloud) / max_points))]
    return cloud


def trajectory(store: Any, tf: Any, world_frame: str, *, every: int = 2) -> NDArray[np.float32]:
    """Where the camera was, from the same thumbnails: one point per frame."""
    name = thumbnail_stream_for("")
    if name not in store.list_streams():
        return np.zeros((0, 3), dtype=np.float32)
    stamps, sources = [], []
    for index, observation in enumerate(store.stream(name, dict).order_by("ts")):
        if index % every:
            continue
        stamps.append(float(observation.data["ts"]))
        sources.append(str(observation.data["camera_frame"]))
    if not stamps:
        return np.zeros((0, 3), dtype=np.float32)
    poses, valid = tf.batch_get(world_frame, sources, stamps)
    return poses[valid][:, :3, 3].astype(np.float32)


def draw_box(image: Any, box: Sequence[float], color: tuple[int, int, int], width: int = 4) -> None:
    from PIL import ImageDraw

    draw = ImageDraw.Draw(image)
    draw.rectangle([box[0], box[1], box[2], box[3]], outline=color, width=width)


def evidence_sheet(
    path: Path,
    query: str,
    detections: Sequence[Detection],
    *,
    columns: int = 3,
    tile_width: int = 640,
) -> Path | None:
    """One tile per detection: the frame the detector saw, its box, and its numbers.

    Only detections that found something get a tile -- a refusal has no picture worth
    looking at, and the caller reports those as counts.
    """
    from PIL import Image as PillowImage, ImageDraw, ImageFont

    found = [d for d in detections if d.found and d.image is not None]
    if not found:
        return None

    banner = 96
    title = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 25)
    small = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 17)
    tiles = []
    for detection in found:
        rgb = np.asarray(detection.image.to_rgb().data)  # type: ignore[union-attr]
        picture = PillowImage.fromarray(rgb)
        assert detection.box2d is not None
        draw_box(picture, detection.box2d, FOUND_COLOR)
        scale = tile_width / picture.width
        picture = picture.resize((tile_width, int(picture.height * scale)))
        tile = PillowImage.new("RGB", (tile_width, picture.height + banner), (16, 17, 24))
        tile.paste(picture, (0, banner))
        draw = ImageDraw.Draw(tile)
        if detection.box3d is None:
            size, where = "no 3D box", detection.note
        else:
            extent = detection.box3d.extent
            size = (
                f"{detection.box3d.depth_m:.1f} m away  ·  "
                f"{extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f} m"
            )
            centre = detection.box3d.centre
            where = f"{detection.box3d.frame} ({centre[0]:.1f}, {centre[1]:.1f}, {centre[2]:.1f})"
        draw.text(
            (14, 6),
            f"#{detection.rank}  {query}   owl {detection.score:.2f}",
            (240, 240, 248),
            font=title,
        )
        draw.text((14, 39), size, (150, 152, 168), font=small)
        draw.text(
            (14, 63),
            f"episode {detection.episode_frames} frames / {detection.episode_span:.1f}s  ·  {where}",
            (150, 152, 168),
            font=small,
        )
        tiles.append(tile)

    rows = int(np.ceil(len(tiles) / columns))
    height = max(tile.height for tile in tiles)
    sheet = PillowImage.new("RGB", (columns * tile_width, rows * height), (10, 11, 16))
    for index, tile in enumerate(tiles):
        sheet.paste(tile, ((index % columns) * tile_width, (index // columns) * height))
    path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(path)
    return path


def _packed(points: NDArray[np.floating]) -> str:
    """A cloud as base64 float32, not as JSON numbers.

    A quarter of a million points written out as decimal text is fifteen megabytes of
    page; the same points as a buffer are one and a half, and the browser gets them
    straight into a `Float32Array` with nothing to parse.
    """
    import base64

    return base64.b64encode(np.ascontiguousarray(points, dtype=np.float32).tobytes()).decode()


def camera_views(
    detections: Sequence[Detection],
    frames: Any,
    world_frame: str,
    *,
    width: int = 384,
    distance: float = 1.2,
) -> list[dict[str, Any]]:
    """The pictures the detector was actually shown, each with where it was taken from.

    A box in the air is an assertion; the frame it came from is the evidence. Placing
    the image on its own view frustum puts the two in the same space, so a box that
    looks wrong can be checked against what the camera saw without leaving the scene.

    The image travels as a small JPEG data URI -- twelve of them at 384 px is a few
    hundred kilobytes, against megabytes for the scene cloud they sit in.
    """
    import base64
    from io import BytesIO

    from PIL import Image as PILImage

    views: list[dict[str, Any]] = []
    for detection in detections:
        if detection.image is None:
            continue
        pose = frames.pose(detection.camera_frame, detection.ts, world_frame)
        intrinsics = frames.intrinsics.get(detection.camera_frame)
        if pose is None or intrinsics is None:
            continue
        rgb = np.asarray(detection.image.to_rgb().data)
        height_px, width_px = rgb.shape[:2]
        picture = PILImage.fromarray(rgb)
        if width_px > width:
            picture = picture.resize((width, max(1, round(height_px * width / width_px))))
        buffer = BytesIO()
        picture.save(buffer, format="JPEG", quality=70)
        # The image plane at `distance`, sized so it subtends what the lens does.
        views.append(
            {
                "rank": detection.rank,
                "place_id": detection.place_id,
                "arrived": detection.arrived,
                "distance": distance,
                "width_m": distance * width_px / float(intrinsics.fx),
                "height_m": distance * height_px / float(intrinsics.fy),
                "pose": [float(v) for v in np.asarray(pose).reshape(-1)],
                "box2d": None if detection.box2d is None else list(detection.box2d),
                "size_px": [width_px, height_px],
                "src": "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode(),
            }
        )
    return views


def boxes_html(
    path: Path,
    query: str,
    detections: Sequence[Detection],
    points: NDArray[np.floating],
    route: NDArray[np.floating],
    *,
    recording: str = "",
    views: Sequence[dict[str, Any]] = (),
    stats: dict[str, Any] | None = None,
) -> Path:
    """An interactive page: the scene in grey, the answers in orange, ranked in a list.

    *stats* is whatever the caller wants said above the list -- how long the query took,
    what it was searched with. It is shown verbatim, in order, so a caller can put its
    own vocabulary on the page rather than one agreed here.
    """
    boxes = [
        {
            "rank": detection.rank,
            "centre": list(detection.box3d.centre),
            "extent": [max(0.05, v) for v in detection.box3d.extent],
            "score": detection.score,
            "depth": detection.box3d.depth_m,
            "ts": detection.ts,
            "frames": detection.episode_frames,
            "span": detection.episode_span,
            "models": detection.models,
            "arrived": detection.arrived,
            "place_id": detection.place_id,
            "refined_centre": None if detection.refined is None else list(detection.refined.centre),
            "refined_extent": None
            if detection.refined is None
            else [max(0.05, v) for v in detection.refined.extent],
            "duplicate_of": detection.duplicate_of,
        }
        for detection in detections
        if detection.box3d is not None
    ]
    refused = [d.rank for d in detections if not d.found]
    # An answer the detector found and nobody could place is not a non-event: it is the
    # one case where the page should say what went wrong, because "sixteen answers" and
    # "sixteen answers and four the depth could not carry" are different results.
    unplaced = [
        {
            "rank": detection.rank,
            "score": detection.score,
            "arrived": detection.arrived,
            "note": detection.note or "not placed",
        }
        for detection in detections
        if detection.found and detection.box3d is None
    ]
    payload = {
        "query": query,
        "recording": recording,
        "boxes": boxes,
        "refused": refused,
        "unplaced": unplaced,
        "stats": stats or {},
        "points": _packed(points),
        "route": _packed(route),
        "views": list(views),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_PAGE.replace("__DATA__", json.dumps(payload, separators=(",", ":"))))
    return path


_PAGE = """<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>hyperspace answers</title>
<style>
:root { --ink:#e8e8ef; --dim:#9a9aa8; --edge:#2a2b36; --panel:#14151c; --hot:#ff8a2e; color-scheme: dark }
* { box-sizing:border-box }
body { margin:0; background:#0a0b10; color:var(--ink); overflow:hidden;
       font:13px/1.5 ui-sans-serif,-apple-system,"Segoe UI",sans-serif; -webkit-text-size-adjust:100% }
/* The canvas claims every touch gesture; without this the browser pans the page
   instead of orbiting the scene. */
#scene { position:fixed; inset:0; touch-action:none }
#side { position:fixed; top:0; right:0; width:330px; height:100%; overflow:auto;
        background:var(--panel); border-left:1px solid var(--edge); padding:16px 18px 24px;
        transition:transform .22s ease; -webkit-overflow-scrolling:touch }
#fold { position:fixed; top:10px; right:10px; z-index:3; cursor:pointer; appearance:none;
        background:var(--panel); color:var(--ink); border:1px solid var(--edge); border-radius:9px;
        padding:9px 13px; font:inherit; font-size:12.5px; min-height:38px;
        transition:transform .22s ease }
#fold b { color:var(--hot) }
body:not(.folded) #fold { transform:translateX(-330px) }
body.folded #side { transform:translateX(100%) }
h1 { font-size:17px; margin:0 0 2px }
h1 span { color:var(--hot) }
.sub { color:var(--dim); margin:0 0 14px; font-size:12px }
.box { border:1px solid var(--edge); border-radius:7px; padding:9px 11px; margin-bottom:8px;
       cursor:pointer; transition:border-color .15s, background .15s }
.box:hover, .box.on { border-color:var(--hot); background:#1b1c25 }
.box.waiting { opacity:.22 }
/* Found, and nowhere to put it. Cool rather than warm: nothing of it is in the scene. */
.box.flat { cursor:default; border-style:dashed; color:var(--dim) }
.box.flat:hover { border-color:var(--edge); background:none }
.box.flat b { color:var(--dim) }
.stats b { color:var(--ink); font-weight:600 }
.box b { color:var(--hot) }
.box .n { color:var(--dim); font-size:11.5px; display:block; margin-top:3px }
.hint { position:fixed; left:14px; bottom:calc(38px + env(safe-area-inset-bottom));
        color:var(--dim); font-size:11.5px; pointer-events:none }
#replay { position:fixed; left:14px; bottom:calc(12px + env(safe-area-inset-bottom));
          display:flex; align-items:center; gap:10px; font-size:11.5px; color:var(--dim) }
#replay button { appearance:none; cursor:pointer; background:var(--panel); color:var(--ink);
                 border:1px solid var(--edge); border-radius:7px; padding:6px 11px;
                 font:inherit; min-height:32px }
#replay button:hover { border-color:var(--hot) }
#clock { font-variant-numeric:tabular-nums; color:var(--hot) }
#track { width:150px; height:4px; border-radius:2px; background:var(--edge); overflow:hidden }
#track i { display:block; height:100%; width:0; background:var(--hot) }
.touch { display:none }
.none { color:var(--dim); font-style:italic }
/* On a phone the list is a sheet off the bottom edge rather than a column that eats
   half the scene, and every tap target grows. */
@media (max-width:720px) {
    #side { top:auto; bottom:0; left:0; width:100%; height:auto; max-height:60dvh;
            border-left:0; border-top:1px solid var(--edge); border-radius:14px 14px 0 0;
            padding:16px 16px calc(20px + env(safe-area-inset-bottom));
            box-shadow:0 -12px 32px rgba(0,0,0,.45) }
    body.folded #side { transform:translateY(101%) }
    body:not(.folded) #fold { transform:none }
    #fold { padding:11px 15px; font-size:14px; min-height:44px }
    .box { padding:12px 13px; font-size:14px; margin-bottom:10px }
    .box .n { font-size:12.5px }
    h1 { font-size:19px }
    .sub { font-size:13px }
    .hint .mouse { display:none }
    .hint .touch { display:inline }
    /* the sheet covers where the hint sits */
    body:not(.folded) .hint { display:none }
}
</style>
<div id="scene"></div>
<button id="fold" type="button"></button>
<div id="side"></div>
<div class="hint"><span class="mouse">drag to orbit &middot; scroll to zoom &middot; shift-drag to pan &middot; click an answer to fly to it</span><span class="touch">drag to orbit &middot; pinch to zoom &middot; two fingers to pan</span></div>
<div id="replay"><button type="button">replay</button><span id="clock">0.00s</span><span id="track"><i></i></span></div>
<script id="data" type="application/json">__DATA__</script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/0.160.0/three.min.js"></script>
<script>
const data = JSON.parse(document.getElementById("data").textContent)
const host = document.getElementById("scene")
// A logarithmic depth buffer because the scenes differ by three orders of magnitude: a
// 20 m room and a 4 km bike ride go through the same page, and a near/far pair wide
// enough for the ride would z-fight its way through the room.
const renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
renderer.setSize(host.clientWidth, host.clientHeight)
host.appendChild(renderer.domElement)

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x0a0b10)
const camera = new THREE.PerspectiveCamera(55, host.clientWidth / host.clientHeight, 0.05, 800)
// `far` is set from the scene once it is known -- see below. 800 m was fine for every
// indoor recording and clipped a bike ride to a black screen.

// The clouds travel as base64 float32 rather than as decimal text: a quarter of a
// million points is 1.5 MB that way and 15 MB written out as JSON numbers.
function unpack(encoded) {
    const binary = atob(encoded)
    const bytes = new Uint8Array(binary.length)
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i)
    return new Float32Array(bytes.buffer)
}

// The scene, as the index saw it: every depth thumbnail placed by its own pose.
const cloud = unpack(data.points)
const geometry = new THREE.BufferGeometry()
geometry.setAttribute("position", new THREE.BufferAttribute(cloud, 3))
// Points sized in pixels rather than in metres. A 22 mm point is a sensible dot in a
// room and invisible along a four-kilometre ride, and there is no one length that
// suits both; two pixels suits both by not being a length at all.
scene.add(new THREE.Points(geometry, new THREE.PointsMaterial({
    size: 2 * Math.min(devicePixelRatio, 2), sizeAttenuation: false, color: 0x5b6070 })))

const route = unpack(data.route)
if (route.length > 5) {
    const points = []
    for (let i = 0; i < route.length; i += 3) points.push(new THREE.Vector3(route[i], route[i+1], route[i+2]))
    scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
              new THREE.LineBasicMaterial({ color: 0x3d6ea5 })))
}

const centre = new THREE.Vector3()
const bounds = new THREE.Box3()
if (cloud.length) {
    geometry.computeBoundingBox()
    bounds.copy(geometry.boundingBox)
} else {
    data.boxes.forEach(b => bounds.expandByPoint(new THREE.Vector3(...b.centre)))
}
bounds.getCenter(centre)
const span = Math.max(bounds.getSize(new THREE.Vector3()).length(), 4)
camera.far = Math.max(800, span * 8)
camera.updateProjectionMatrix()

// Lit rather than flat, so a solid face still reads as a face and the boxes look
// like objects sitting in the room instead of six lines.
scene.add(new THREE.HemisphereLight(0xffffff, 0x14151c, 2.0))
const sun = new THREE.DirectionalLight(0xffffff, 1.2)
sun.position.set(1, 1.5, 2)
scene.add(sun)

// --- the camera frames the detector was shown ------------------------------------
// Each one hangs on its own view frustum, so the evidence for a box sits in the same
// space as the box: a wrong answer can be checked against what the camera saw.
const views = (data.views || []).map(view => {
    const texture = new THREE.TextureLoader().load(view.src)
    texture.colorSpace = THREE.SRGBColorSpace
    const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(view.width_m, view.height_m),
        new THREE.MeshBasicMaterial({ map: texture, transparent: true, opacity: 0.92,
                                      side: THREE.DoubleSide, depthWrite: false }))
    // The pose is row-major and the optical frame has y pointing down, so the plane is
    // flipped in y to keep the picture the right way up.
    const pose = new THREE.Matrix4().fromArray(view.pose).transpose()
    const place = new THREE.Matrix4().makeTranslation(0, 0, view.distance)
    const flip = new THREE.Matrix4().makeScale(1, -1, 1)
    plane.matrixAutoUpdate = false
    plane.matrix.copy(pose).multiply(place).multiply(flip)
    plane.visible = false
    scene.add(plane)

    // A line back to where the camera stood, so a frame far from its box is obvious.
    const eye = new THREE.Vector3().setFromMatrixPosition(pose)
    const corner = new THREE.Vector3()
    const edges = []
    for (const [sx, sy] of [[-1,-1],[1,-1],[1,1],[-1,1]]) {
        corner.set(sx * view.width_m / 2, sy * view.height_m / 2, 0).applyMatrix4(plane.matrix)
        edges.push(eye.clone(), corner.clone())
    }
    const frustum = new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(edges),
        new THREE.LineBasicMaterial({ color: 0x4b5168, transparent: true, opacity: 0.5 }))
    frustum.visible = false
    scene.add(frustum)
    return { plane, frustum, eye, rank: view.rank, arrived: Number(view.arrived) || 0 }
})

// Which photograph belongs to which answer, so clicking an answer can go and stand
// where the detector stood rather than somewhere the evidence happens to be edge-on.
const viewOfRank = new Map(views.map(view => [view.rank, view]))

const shapes = data.boxes.map(box => {
    // A second look at a place already found is drawn cooler, so the distinct answers
    // are the ones that stand out.
    // A refinement is the same place, so it keeps the place's colour; only the flare
    // as it lands says that something changed.
    const tone = 0xff8a2e
    // How solid a box looks is how sure the detector was. A guess is a haze you can
    // see straight through; a confident answer is nearly opaque.
    const sure = Math.max(0, Math.min(1, box.score))
    // The place's box as of this answer, which for a second look is the average of the
    // looks so far rather than a new box beside the old one.
    const centre = box.refined_centre || box.centre
    const extent = box.refined_extent || box.extent
    const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(...extent),
        new THREE.MeshLambertMaterial({
            color: tone, transparent: true, opacity: 0.10 + 0.55 * sure,
            // Without this a weak box in front hides a strong one behind it.
            depthWrite: false, side: THREE.DoubleSide }))
    mesh.position.set(...centre)
    mesh.renderOrder = 1 + sure
    const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(mesh.geometry),
        new THREE.LineBasicMaterial({ color: tone, transparent: true, opacity: 0.35 + 0.6 * sure }))
    edges.position.copy(mesh.position)
    scene.add(mesh); scene.add(edges)
    // A pixel-sized dot at the same place. A metre-wide box is a couple of pixels
    // across a four-kilometre bike ride, so without this the overview of a long
    // recording shows the route and none of the answers on it; the dot does not
    // shrink, so where the answers are is legible before you have flown to one.
    const dot = new THREE.Points(
        new THREE.BufferGeometry().setAttribute("position",
            new THREE.Float32BufferAttribute([...centre], 3)),
        new THREE.PointsMaterial({ size: 7 * Math.min(devicePixelRatio, 2),
                                   sizeAttenuation: false, color: tone,
                                   depthTest: false, transparent: true }))
    dot.renderOrder = 3
    scene.add(dot)
    return { mesh, edges, dot, solid: 0.10 + 0.55 * sure, line: 0.35 + 0.6 * sure,
             place: box.place_id == null ? `rank${box.rank}` : `place${box.place_id}` }
})
const drawn = shapes.map(shape => shape.mesh)

// --- the answers arriving ---------------------------------------------------------
// Each box carries the moment the module handed it back, so the page shows the query
// happening rather than its result: the first answer lands in a fraction of a second
// and the rest follow one detector call apart.
const arrivals = data.boxes.map(box => Number(box.arrived) || 0)
const lastArrival = Math.max(0.001, ...arrivals)
const clock = document.getElementById("clock")
const progress = document.querySelector("#track i")
let playedFrom = null

function showUpTo(seconds) {
    views.forEach(view => {
        view.plane.visible = view.frustum.visible = seconds >= view.arrived
    })
    // One box per place, not per answer: a second look at somewhere already found moves
    // the box it already has rather than stacking another on top of it.
    const owner = new Map()
    shapes.forEach((shape, index) => {
        if (seconds >= arrivals[index]) owner.set(shape.place, index)
    })
    shapes.forEach((shape, index) => {
        const since = seconds - arrivals[index]
        const shown = since >= 0 && owner.get(shape.place) === index
        shape.mesh.visible = shape.edges.visible = shape.dot.visible = shown
        if (!shown) return
        // A box flares as it arrives and settles into its confidence, so a new answer
        // is visible even when the camera is somewhere else.
        const flare = Math.max(0, 1 - since / 0.6)
        shape.mesh.material.opacity = Math.min(1, shape.solid + 0.35 * flare)
        shape.edges.material.opacity = Math.min(1, shape.line + 0.4 * flare)
    })
    const cards = document.querySelectorAll(".box.placed")
    cards.forEach((card, index) => {
        card.classList.toggle("waiting", seconds < arrivals[index])
    })
    clock.textContent = `${Math.min(seconds, lastArrival).toFixed(2)}s`
    progress.style.width = `${Math.min(100, (seconds / lastArrival) * 100)}%`
}

function replay() {
    playedFrom = performance.now()
}
document.querySelector("#replay button").onclick = replay

// --- an orbit camera, written out rather than pulled in ---------------------------
let yaw = 0.6, pitch = 0.9, range = span * 0.75
let target = centre.clone()
function place() {
    camera.position.set(
        target.x + range * Math.sin(pitch) * Math.cos(yaw),
        target.y + range * Math.sin(pitch) * Math.sin(yaw),
        target.z + range * Math.cos(pitch))
    camera.up.set(0, 0, 1)
    camera.lookAt(target)
}
place()

function pan(dx, dy) {
    const view = new THREE.Vector3().subVectors(camera.position, target).normalize()
    const right = new THREE.Vector3().crossVectors(view, camera.up).normalize()
    const up = new THREE.Vector3().crossVectors(right, view).normalize()
    target.addScaledVector(right, -dx * range * 0.0015)
    target.addScaledVector(up, dy * range * 0.0015)
    place()
}
function zoom(factor) {
    range = Math.min(span * 4, Math.max(0.5, range * factor))
    place()
}

// One finger orbits, two fingers pinch and pan -- the same code serves a mouse, where
// the second finger is the shift key.
const touches = new Map()
const surface = renderer.domElement
function spread() {
    if (touches.size < 2) return 0
    const [a, b] = [...touches.values()]
    return Math.hypot(a.x - b.x, a.y - b.y)
}
function middle() {
    const all = [...touches.values()]
    return { x: all.reduce((s, p) => s + p.x, 0) / all.length,
             y: all.reduce((s, p) => s + p.y, 0) / all.length }
}
surface.addEventListener("pointerdown", event => {
    surface.setPointerCapture(event.pointerId)
    touches.set(event.pointerId, { x: event.clientX, y: event.clientY })
})
const lift = event => touches.delete(event.pointerId)
surface.addEventListener("pointerup", lift)
surface.addEventListener("pointercancel", lift)
surface.addEventListener("pointermove", event => {
    const held = touches.get(event.pointerId)
    if (!held) return
    const wasSpread = spread(), wasMiddle = middle()
    held.x = event.clientX; held.y = event.clientY
    const nowSpread = spread(), nowMiddle = middle()
    const dx = nowMiddle.x - wasMiddle.x, dy = nowMiddle.y - wasMiddle.y
    if (touches.size > 1) {
        if (wasSpread > 0 && nowSpread > 0) zoom(wasSpread / nowSpread)
        pan(dx, dy)
    } else if (event.shiftKey) {
        pan(dx, dy)
    } else {
        yaw -= dx * 0.006
        pitch = Math.min(Math.PI - 0.05, Math.max(0.05, pitch - dy * 0.006))
        place()
    }
})
surface.addEventListener("wheel", event => {
    event.preventDefault()
    zoom(1 + Math.sign(event.deltaY) * 0.12)
}, { passive: false })
function fit() {
    camera.aspect = host.clientWidth / host.clientHeight
    camera.updateProjectionMatrix()
    renderer.setSize(host.clientWidth, host.clientHeight)
}
addEventListener("resize", fit)
addEventListener("orientationchange", fit)
;(function draw() {
    requestAnimationFrame(draw)
    if (playedFrom !== null) {
        const seconds = (performance.now() - playedFrom) / 1000
        showUpTo(seconds)
        if (seconds > lastArrival + 1.0) playedFrom = null
    }
    renderer.render(scene, camera)
})()

// --- the list ---------------------------------------------------------------------
const side = document.getElementById("side")
const title = document.createElement("h1")
title.innerHTML = `<span>${data.query}</span>`
side.appendChild(title)
const sub = document.createElement("p")
sub.className = "sub"
const places = data.boxes.filter(b => !b.duplicate_of).length
sub.textContent = `${data.boxes.length} placed in ${places} distinct place(s)` +
    (data.refused.length ? `, ${data.refused.length} episode(s) the detector refused` : "") +
    (data.recording ? ` · ${data.recording}` : "")
side.appendChild(sub)
const stats = Object.entries(data.stats || {})
if (stats.length) {
    const line = document.createElement("p")
    line.className = "sub stats"
    line.innerHTML = stats.map(([name, value]) => `${name} <b>${value}</b>`).join(" &middot; ")
    side.appendChild(line)
}
const legend = document.createElement("p")
legend.className = "sub"
legend.textContent = "A box is as solid as the detector was sure; a faint one is a guess."
side.appendChild(legend)
if (!data.boxes.length) {
    const none = document.createElement("p")
    none.className = "none"
    none.textContent = "Nothing was placed in 3D."
    side.appendChild(none)
}

// The list starts out of the way on a phone, where it would otherwise cover the thing
// it is describing.
const narrow = () => innerWidth <= 720
const fold = document.getElementById("fold")
function setFolded(shut) {
    document.body.classList.toggle("folded", shut)
    fold.innerHTML = shut ? `<b>${data.boxes.length}</b> answer${data.boxes.length === 1 ? "" : "s"}` : "hide"
    fold.setAttribute("aria-expanded", String(!shut))
}
fold.onclick = () => setFolded(!document.body.classList.contains("folded"))
setFolded(narrow())

data.boxes.forEach((box, index) => {
    const card = document.createElement("div")
    card.className = "box placed"
    card.innerHTML = `<b>#${box.rank}</b> owl ${box.score.toFixed(2)} &middot; ${box.depth.toFixed(1)} m away` +
        (box.duplicate_of ? `<span class="n">another look at #${box.duplicate_of}</span>` : "") +
        `<span class="n">${box.centre.map(v => v.toFixed(1)).join(", ")} m &middot; ` +
        `${box.extent.map(v => v.toFixed(2)).join(" x ")} m</span>` +
        `<span class="n">${box.frames} frames over ${box.span.toFixed(1)}s &middot; ${box.models.join(", ")}</span>`
    card.onclick = () => {
        document.querySelectorAll(".box.placed").forEach(el => el.classList.remove("on"))
        card.classList.add("on")
        target = drawn[index].position.clone()
        range = Math.max(1.5, Math.max(...box.extent) * 6)
        // Point at the box and you can easily end up behind its photograph, or looking
        // at it edge-on -- a plane has no thickness. So take the direction from the
        // answer's own frame instead: stand behind where that camera stood and a little
        // above it, looking the way it looked. The picture is then nearly face-on with
        // the box beyond it, which is the whole point of hanging the two in one space.
        const seen = viewOfRank.get(box.rank)
        if (seen) {
            // Flying to a photograph the clock has not reached yet shows an empty frustum,
            // so stop the replay and settle it at the moment this answer arrived.
            playedFrom = null
            showUpTo(Math.max(arrivals[index], seen.arrived))
            const along = new THREE.Vector3().subVectors(target, seen.eye)
            const reach = along.length()
            if (reach > 1e-3) {
                along.divideScalar(reach)
                const back = Math.max(2.5, reach * 0.6)
                const stand = seen.eye.clone().addScaledVector(along, -back)
                stand.z += back * 0.35
                const offset = new THREE.Vector3().subVectors(stand, target)
                range = Math.max(1.5, offset.length())
                const unit = offset.normalize()
                pitch = Math.acos(Math.min(1, Math.max(-1, unit.z)))
                yaw = Math.atan2(unit.y, unit.x)
            }
        }
        place()
        // Flying somewhere you cannot see is no use: on a phone the sheet gets out of
        // the way once it has been used.
        if (narrow()) setFolded(true)
    }
    side.appendChild(card)
})

// Found by the detector, and nowhere to put it. Worth a line each: "the depth image
// has no reading inside the box" and "every reading is past the 10 m cut-off" are
// different faults, and a page that only counted them would hide which one this was.
if ((data.unplaced || []).length) {
    const heading = document.createElement("p")
    heading.className = "sub"
    heading.style.marginTop = "14px"
    heading.textContent = `${data.unplaced.length} found but not placed`
    side.appendChild(heading)
    data.unplaced.forEach(answer => {
        const card = document.createElement("div")
        card.className = "box flat"
        card.innerHTML = `<b>#${answer.rank}</b> owl ${answer.score.toFixed(2)}` +
            `<span class="n">${answer.note}</span>`
        side.appendChild(card)
    })
}

// Show the query happening once, on arrival, rather than presenting the finished
// answer as though it were free.
replay()
</script>
"""
