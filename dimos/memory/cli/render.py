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
observation, so streams stay aligned). Writes a ``.rrd`` and opens the viewer.

``rerun_config`` takes the same shape the live viewer's
:class:`~dimos.visualization.rerun.bridge.RerunBridgeModule` already accepts, so
a recording renders the way the robot looked while it was being made rather than
through a second, separately-invented set of conventions. Omitting it leaves the
behaviour here exactly as it was.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


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
    rerun_config: dict[str, Any] | None = None,
) -> str:
    """Render ``store`` to a ``.rrd`` and (unless ``no_gui``) open the rerun viewer.

    Logs every observation (full res); ``seconds`` bounds the time window from
    the start. ``root`` nests every stream under that entity path
    (``<root>/<name>``) — except a stream whose name matches ``root``'s last
    segment, which stays at ``<root>`` itself. Returns the ``.rrd`` path.

    Args:
        rerun_config: Optional, and keyed by entity path exactly as the live
            bridge's config is, so a robot's existing viewer config renders its
            own recordings unchanged. Recognised keys:

            ``visual_override``
                ``{entity_path: fn(payload)}``, replacing that stream's own
                ``to_rerun``. This is how a camera gets a frustum: the intrinsics
                convert to an ``rr.Pinhole`` logged at the *image's* path.
            ``static``
                ``{entity_path: fn(rr)}``, logged once with ``static=True`` so it
                is present at every instant. For fixed geometry only -- a robot
                body, a floor plan. Anything the robot had to observe does not
                belong here, because static means visible before it was seen.
            ``blueprint``
                ``fn() -> BlueprintLike``, saved as the recording's default
                layout.
            ``tf_axes``
                Axis length; any value above zero nests the tf frames into their
                real tree and attaches each stream carrying a ``frame_id`` to its
                frame. Left at zero the entity paths stay flat, which is what
                every caller before this argument existed got.
    """
    import rerun as rr

    from dimos.memory.utils.progress import progress
    from dimos.visualization.rerun.init import rerun_init

    config = rerun_config or {}
    overrides: dict[str, Any] = config.get("visual_override") or {}
    statics: dict[str, Any] = config.get("static") or {}
    blueprint = config.get("blueprint")
    tf_axes = float(config.get("tf_axes") or 0.0)

    tf_tree = None
    if tf_axes > 0:
        from dimos.msgs.tf2_msgs.TFMessage import TfFrameTree

        tf_tree = TfFrameTree(axis_length=tf_axes)

    if out is None:
        src = getattr(store.config, "path", None) or "store"
        out = str(Path(src).with_suffix(".rrd"))

    base = root.strip("/") if root else ""

    def entity(name: str) -> str:
        # <root>/<name>, but a stream named like root's last segment stays at <root>.
        if not base:
            return name
        return base if name == base.rsplit("/", 1)[-1] else f"{base}/{name}"

    def convert(path: str, payload: Any) -> tuple[Any, bool]:
        """Convert one payload, and say whether it named absolute entity paths.

        An override is written against entity paths, the way the live bridge's
        config is, so the paths it returns are final. A payload's own
        ``to_rerun`` cannot know where its stream was mounted, so the parts it
        names are nested under it.
        """
        fn = overrides.get(path)
        if fn is not None:
            return fn(payload), True
        return payload.to_rerun(), False

    # Discover renderable streams (payload has a working to_rerun) + shared anchor.
    renderable = []
    t0: float | None = None
    for name in store.list_streams():
        stream = store.streams[name]
        try:
            first = stream.first()
        except LookupError:
            continue
        data = first.data
        path = entity(name)
        if path not in overrides and not hasattr(data, "to_rerun"):
            print(f"  skip {name}: {type(data).__name__} has no to_rerun()")
            continue
        try:
            convert(path, data)
        except Exception as e:
            print(f"  skip {name}: to_rerun() failed ({e})")
            continue
        renderable.append((name, stream))
        t0 = first.ts if t0 is None else min(t0, first.ts)

    if t0 is None:
        print("nothing renderable in this store")
        return out

    rerun_init("dimos mem rerun")
    rr.save(out, default_blueprint=blueprint() if blueprint else None)

    for path, factory in statics.items():
        archetypes = factory(rr)
        for archetype in archetypes if isinstance(archetypes, list) else [archetypes]:
            # A factory may name its own path, as the bridge's statics do.
            if isinstance(archetype, tuple):
                rr.log(archetype[0], archetype[1], static=True)
            else:
                rr.log(path, archetype, static=True)

    attached: dict[str, str] = {}
    for name, stream in renderable:
        path = entity(name)
        with progress(stream.count(), label=name) as report:
            for obs in stream:
                if seconds is not None and obs.ts - t0 > seconds:
                    break  # the context manager finalizes the windowed (sub-100%) bar
                if obs.data is None:  # e.g. a truncated/corrupt frame that failed to decode
                    report(obs)
                    continue
                rr.set_time("time", duration=obs.ts - t0)
                if tf_tree is not None and hasattr(obs.data, "transforms"):
                    data, absolute = obs.data.to_rerun(tf_tree), False  # nest frames under parents
                else:
                    data, absolute = convert(path, obs.data)
                if isinstance(data, list):  # RerunMulti: [(subpath, archetype), ...]
                    for sub, arch in data:
                        rr.log(sub if absolute else f"{path}/{sub}", arch)
                else:
                    rr.log(path, data)
                    # Carrying a frame_id means the payload is expressed in that
                    # frame; attaching it once puts the image inside the camera's
                    # frustum instead of at the world origin.
                    # A static factory that positions its own entity owns that
                    # path's parenting; attaching it again by frame_id would give
                    # the frame two parents, which rerun rejects outright.
                    if (
                        tf_tree is not None
                        and not isinstance(data, rr.Transform3D)
                        and path not in statics
                    ):
                        frame_id = getattr(obs.data, "frame_id", None)
                        if frame_id and attached.get(path) != frame_id:
                            rr.log(path, rr.Transform3D(parent_frame=f"tf#/{frame_id}"))
                            attached[path] = frame_id
                report(obs)

    rr.rerun_shutdown()  # flush + close the .rrd before opening it
    print(f"wrote {out}")
    if not no_gui:
        _open_viewer(out)
    return out
