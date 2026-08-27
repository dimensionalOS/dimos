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

"""The recording seam: typed streams, declarations, and the reader protocol.

The wire format is its own axis of variation, separate from the engine and
the robot's physics: a different robot means different topics, message
classes and motor orderings, and none of that belongs to the pipeline. So
the pipeline consumes ONE currency — :class:`Streams` — and a
:class:`RecordingReader` is whatever turns a file into it. The Go2 DDS
reader (:mod:`~dimos.robot.unitree.go2.sim.sysid.ingest`) is one
implementation; a second robot brings a reader, not a fork of ingest.

Readers are PICKLABLE, like backends: worker processes re-read the recording
themselves (cheap — the parent's read warmed the cache), so the reader
travels to them by pickle.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Protocol

import numpy as np

from dimos.robot.unitree.go2.sim.rotations import quat_to_mat

# The MCAP file-level metadata record declarations are read from.
METADATA_KEY = "go2sim"


@dataclass
class Streams:
    """Every recorded signal the pipeline needs, on the first-walk-command epoch.

    The common currency between a recording and everything above it: readers
    produce it, nothing downstream knows a topic name or a wire order. The
    tracker is OPTIONAL — detect it and use it if present; a bare robot
    walked around a room is a first-class input.
    """

    lt: np.ndarray  # lowstate time, s (robot clock, rebased)
    lq: np.ndarray  # (n,12) measured joint angles, actuator order
    ldq: np.ndarray  # (n,12) measured joint speeds
    ltau: np.ndarray  # (n,12) motor-side torque estimate
    lquat: np.ndarray  # (n,4) IMU attitude, wxyz
    lgyro: np.ndarray  # (n,3) body angular rate
    lacc: np.ndarray  # (n,3) body specific force — ~0 in free fall
    ct: np.ndarray  # command time, s
    cq: np.ndarray  # (m,12) COMMANDED joint targets — the replay input
    ckp: np.ndarray
    ckd: np.ndarray
    ctau: np.ndarray  # feed-forward torque (zero from our own executor)
    cdq: np.ndarray  # (m,12) COMMANDED joint speeds; nonzero from Unitree's builtins
    vt: np.ndarray = field(default_factory=lambda: np.zeros(0))  # tracker time, s
    vp: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    vq: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
    seg_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    seg_mode: tuple[str, ...] = ()
    # Operator command schedule (velocity commands): what drives the policy
    # closed loop in Mode B, and the cmd axis of loop 2's statistics.
    wt: np.ndarray = field(default_factory=lambda: np.zeros(0))  # walk cmd time, s
    wcmd: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # vx, vy, vyaw
    # Gait-height slider; empty when never touched.
    ght: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gh: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def has_markers(self) -> bool:
        return len(self.vt) > 0

    def segments(self) -> list[tuple[int, str, float, float]]:
        """``(index, policy mode, t_start, t_end)`` for each policy span."""
        end = float(self.ct[-1])
        out = []
        for i, mode in enumerate(self.seg_mode):
            a = float(self.seg_t[i])
            b = float(self.seg_t[i + 1]) if i + 1 < len(self.seg_t) else end
            out.append((i, mode, max(a, float(self.ct[0])), min(b, end)))
        return out

    def base_pose_room(self, mount: np.ndarray, tracker_z: float) -> tuple[np.ndarray, np.ndarray]:
        """Measured base position and rotation in the ROOM frame, from the tracker.

        ``mount`` maps base-frame vectors into the tracker frame and
        ``tracker_z`` is the mount's vertical lever arm — both properties of
        the rig, supplied by the robot's side (Go2:
        :func:`~dimos.robot.unitree.go2.sim.sysid.ingest.mount_matrix`).
        """
        rot = quat_to_mat(self.vq)  # tracker -> room
        base_r = np.einsum("nij,jk->nik", rot, mount.T)  # base -> room
        base_p = self.vp - base_r @ np.array([0.0, 0.0, tracker_z])
        return base_p, base_r


@dataclass(frozen=True)
class Declarations:
    """The two things no signal can reveal; everything else is measured.

    A hanging robot reads ~1 g with unloaded legs — and so does one lying
    down: suspension is ambiguous in principle. Floor material is in no
    recorded signal at all. So exactly these two are DECLARED, in order of
    authority: an MCAP file-level :data:`METADATA_KEY` metadata record
    written at capture time, else a ``<recording>.meta.json`` sidecar. A
    detector may cross-check a declaration; it never decides.
    """

    suspended: bool | None = None
    floor: str | None = None


def sidecar_path(recording: str | Path) -> Path:
    return Path(recording).with_suffix(".meta.json")


def read_declarations(path: str | Path) -> Declarations:
    path = Path(path)
    from mcap.reader import make_reader

    with path.open("rb") as f:
        for md in make_reader(f).iter_metadata():
            if md.name != METADATA_KEY:
                continue
            kv = dict(md.metadata)
            return Declarations(
                suspended=json.loads(kv["suspended"]) if "suspended" in kv else None,
                floor=kv.get("floor"),
            )
    side = sidecar_path(path)
    if side.is_file():
        d = json.loads(side.read_text())
        return Declarations(suspended=d.get("suspended"), floor=d.get("floor"))
    return Declarations()


class RecordingReader(Protocol):
    """Turns one recording file into :class:`Streams`. Picklable, like a backend."""

    @property
    def cache_tag(self) -> str:
        """Names the reader AND its parse version in the cache key. Bump the
        version whenever the field set or the parse changes: a stale cache
        would otherwise hide a reader's fixes behind an old npz."""
        ...

    def read(self, path: Path) -> Streams:
        """Parse the file. Called on a cache miss; may take tens of seconds."""
        ...


def _cache_path(path: Path, tag: str) -> Path:
    d = Path.home() / ".cache" / "dimos_go2sim"
    d.mkdir(parents=True, exist_ok=True)
    stat = path.stat()
    return d / f"{path.stem}.{tag}.{int(stat.st_mtime)}.{stat.st_size}.npz"


def read_recording(path: str | Path, reader: RecordingReader, *, cache: bool = True) -> Streams:
    """One recording through ``reader``, cached under ``~/.cache/dimos_go2sim``.

    The cache is keyed by path, mtime, size and the reader's
    :attr:`~RecordingReader.cache_tag`, and stores the PARSED streams — so
    the npz layer is format-agnostic and every reader shares it.
    """
    path = Path(path)
    cp = _cache_path(path, reader.cache_tag) if cache else None
    if cp is not None and cp.is_file():
        z = np.load(cp, allow_pickle=False)
        arrays = {k: z[k] for k in z.files if k != "seg_mode"}
        return Streams(seg_mode=tuple(str(m) for m in z["seg_mode"]), **arrays)
    st = reader.read(path)
    if cp is not None:
        arrays = {
            k: v for k, v in vars(st).items() if k != "seg_mode" and isinstance(v, np.ndarray)
        }
        np.savez_compressed(cp, seg_mode=np.array(st.seg_mode, dtype="U32"), **arrays)
    return st
