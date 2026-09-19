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

"""Render any memory store into rerun.

Generic: walks the store's streams and logs every observation whose payload
implements ``to_rerun()`` (the :class:`RerunConvertible` convention). Streams
whose payload has no ``to_rerun`` are skipped. Each stream becomes an entity
path; observations share one ``time`` timeline (relative to the store's earliest
observation, so streams stay aligned). CameraInfo streams are the exception:
logged once as a Pinhole on their matching image entity (see
:func:`_pair_camera_infos`). Writes a ``.rrd`` and opens the viewer.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


def _pair_camera_infos(
    renderable: list[tuple[str, Stream[Any], Observation[Any]]],
) -> tuple[dict[str, tuple[CameraInfo, str]], set[str]]:
    """Match each CameraInfo stream to the Image stream(s) it calibrates.

    Pairs by ``frame_id``, falling back to the only image stream when the store
    has one of each; the image's frame_id wins as the pinhole's parent frame.
    Returns (image stream -> (CameraInfo, parent frame), paired stream names).
    """
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

    infos = [(n, f.data) for n, _, f in renderable if isinstance(f.data, CameraInfo)]
    images = [(n, f.data) for n, _, f in renderable if isinstance(f.data, Image)]

    pinholes: dict[str, tuple[CameraInfo, str]] = {}
    paired: set[str] = set()
    for info_name, info in infos:
        targets = [(n, img) for n, img in images if img.frame_id and img.frame_id == info.frame_id]
        if not targets and len(infos) == 1 and len(images) == 1:
            targets = [images[0]]
            print(
                f"  {info_name}: frame_id {info.frame_id!r} matches no image stream, "
                f"pairing with {targets[0][0]!r} as the only image stream"
            )
        if not targets:
            print(f"  {info_name}: no image stream matches frame_id {info.frame_id!r}")
            continue
        for target, img in targets:
            pinholes[target] = (info, img.frame_id or info.frame_id)
        paired.add(info_name)
    return pinholes, paired


def _frame_first_seen(
    renderable: list[tuple[str, Stream[Any], Observation[Any]]], frames: set[str]
) -> dict[str, float]:
    """ts of the first tf observation that defines each frame in ``frames``.

    A Pinhole parented to a tf frame resolves at the origin until that frame
    exists, so it must be logged when its own frame first appears, not at the
    first tf message overall (which may be an unrelated or static edge).
    Scans each tf stream only until every frame is found.
    """
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    seen: dict[str, float] = {}
    for _, stream, first in renderable:
        if not isinstance(first.data, TFMessage):
            continue
        missing = set(frames)
        for obs in stream:
            if not missing:
                break
            if obs.data is None:
                continue
            for t in obs.data.transforms:
                if t.child_frame_id in missing:
                    missing.discard(t.child_frame_id)
                    seen[t.child_frame_id] = min(obs.ts, seen.get(t.child_frame_id, obs.ts))
    return seen


def _open_viewer(rrd: str) -> None:
    exe = shutil.which("rerun")
    if exe:
        subprocess.Popen([exe, rrd])
        print(f"  opening {rrd} in rerun")
    else:
        print(f"  rerun viewer not found on PATH; open manually:\n    rerun {rrd}")


def render_store(
    store: Store,
    *,
    out: str | None = None,
    seconds: float | None = None,
    no_gui: bool = False,
    root: str | None = None,
) -> str:
    """Render ``store`` to a ``.rrd`` and (unless ``no_gui``) open the rerun viewer.

    Logs every observation (full res); ``seconds`` bounds the time window from
    the start. ``root`` nests every stream under that entity path
    (``<root>/<name>``) — except a stream whose name matches ``root``'s last
    segment, which stays at ``<root>`` itself. Returns the ``.rrd`` path.
    """
    import rerun as rr

    from dimos.memory.utils.progress import progress
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.visualization.rerun.init import rerun_init

    if out is None:
        src = getattr(store.config, "path", None) or "store"
        out = str(Path(src).with_suffix(".rrd"))

    base = root.strip("/") if root else ""

    def entity(name: str) -> str:
        # <root>/<name>, but a stream named like root's last segment stays at <root>.
        if not base:
            return name
        return base if name == base.rsplit("/", 1)[-1] else f"{base}/{name}"

    # Discover renderable streams (payload has a working to_rerun) + shared anchor.
    renderable: list[tuple[str, Stream[Any], Observation[Any]]] = []
    for name in store.list_streams():
        stream = store.streams[name]
        try:
            first = stream.first()
        except LookupError:
            continue
        data = first.data
        if not hasattr(data, "to_rerun"):
            print(f"  skip {name}: {type(data).__name__} has no to_rerun()")
            continue
        try:
            data.to_rerun()
        except Exception as e:
            print(f"  skip {name}: to_rerun() failed ({e})")
            continue
        renderable.append((name, stream, first))

    # Paired camera_info streams are logged once below; keep their stale ts out of t0.
    pinholes, paired = _pair_camera_infos(renderable)
    renderable = [entry for entry in renderable if entry[0] not in paired]

    t0 = min((first.ts for _, _, first in renderable), default=None)
    if t0 is None:
        print("nothing renderable in this store")
        return out

    # Log each pinhole when its own parent frame first appears in tf: before
    # that the frame is undefined and the pinhole would sit at the origin.
    # Static only when the store has no tf at all.
    has_tf = any(isinstance(f.data, TFMessage) for _, _, f in renderable)
    frame_at = _frame_first_seen(renderable, {frame for _, frame in pinholes.values()})

    rerun_init("dimos mem rerun")
    rr.save(out)

    for image_name, (info, frame) in pinholes.items():
        pinhole = info.to_rerun_pinhole(optical_frame=frame)
        at = frame_at.get(frame)
        if not has_tf:
            rr.log(entity(image_name), pinhole, static=True)
            print(f"  {image_name}: pinhole from camera_info (frame {frame!r}, static: no tf)")
        elif at is None:
            print(f"  {image_name}: pinhole skipped, frame {frame!r} never appears in tf")
        elif seconds is not None and at - t0 > seconds:
            print(
                f"  {image_name}: pinhole skipped, frame {frame!r} first appears after the window"
            )
        else:
            rr.set_time("time", duration=at - t0)
            rr.log(entity(image_name), pinhole)
            print(f"  {image_name}: pinhole from camera_info (frame {frame!r} at +{at - t0:.2f}s)")

    for name, stream, _ in renderable:
        with progress(stream.count(), label=name) as report:
            for obs in stream:
                if seconds is not None and obs.ts - t0 > seconds:
                    break  # the context manager finalizes the windowed (sub-100%) bar
                if obs.data is None:  # e.g. a truncated/corrupt frame that failed to decode
                    report(obs)
                    continue
                rr.set_time("time", duration=obs.ts - t0)
                data = obs.data.to_rerun()
                path = entity(name)
                if isinstance(data, list):  # RerunMulti: [(subpath, archetype), ...]
                    for sub, arch in data:
                        rr.log(f"{path}/{sub}", arch)
                else:
                    rr.log(path, data)
                report(obs)

    rr.rerun_shutdown()  # flush + close the .rrd before opening it
    print(f"wrote {out}")
    if not no_gui:
        _open_viewer(out)
    return out
