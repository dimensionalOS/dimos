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

"""Persistent environment state and named-coordinate handoff for streamed bodies."""

from collections import OrderedDict
from collections.abc import Iterable
import json
from pathlib import Path
from queue import Queue
import sqlite3
import threading
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.experimental.gta_sim2.world_index import instance_number

OBJECT_PREFIX = "gta_object/"


class ObjectStateStore:
    """One world-state writer; bounded queues keep disk work off the physics owner."""

    def __init__(self, path: Path, catalog_digest: str) -> None:
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(path) as db:
            db.execute("PRAGMA journal_mode=WAL")
            db.execute("CREATE TABLE IF NOT EXISTS identity(digest TEXT PRIMARY KEY)")
            existing = db.execute("SELECT digest FROM identity").fetchone()
            if existing and existing[0] != catalog_digest:
                raise ValueError("Persistent state belongs to a different world catalog")
            db.execute("INSERT OR IGNORE INTO identity VALUES (?)", (catalog_digest,))
            db.execute(
                "CREATE TABLE IF NOT EXISTS objects(id TEXT PRIMARY KEY,number INTEGER UNIQUE,state TEXT NOT NULL)"
            )
            db.execute(
                "CREATE VIRTUAL TABLE IF NOT EXISTS locations USING rtree(number,minx,maxx,miny,maxy,minz,maxz)"
            )
            db.execute("CREATE TABLE IF NOT EXISTS generation(value INTEGER NOT NULL)")
            if not db.execute("SELECT 1 FROM generation").fetchone():
                db.execute("INSERT INTO generation VALUES (0)")
        self._queue: Queue[list[dict[str, Any]] | None] = Queue(4)
        self._latest: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._error: Exception | None = None
        self._thread = threading.Thread(target=self._write, name="world-state-writer", daemon=True)
        self._thread.start()

    def _write(self) -> None:
        try:
            with sqlite3.connect(self.path) as db:
                while True:
                    batch = self._queue.get()
                    if batch is None:
                        return
                    with db:
                        for row in batch:
                            number = instance_number(row["identity"])
                            db.execute(
                                "INSERT INTO objects VALUES (?,?,?) ON CONFLICT(id) DO UPDATE SET state=excluded.state",
                                (row["identity"], number, json.dumps(row, allow_nan=False)),
                            )
                            p, radius = (
                                np.asarray(row["position"], dtype=np.float64),
                                float(row["radius"]),
                            )
                            db.execute(
                                "INSERT OR REPLACE INTO locations VALUES (?,?,?,?,?,?,?)",
                                (
                                    number,
                                    p[0] - radius,
                                    p[0] + radius,
                                    p[1] - radius,
                                    p[1] + radius,
                                    p[2] - radius,
                                    p[2] + radius,
                                ),
                            )
                        db.execute("UPDATE generation SET value=value+1")
        except Exception as error:
            self._error = error

    def save(self, snapshots: list[dict[str, Any]]) -> None:
        if self._error:
            raise RuntimeError("Persistent state writer failed") from self._error
        if not snapshots:
            return
        # No silent state loss if storage is stalled. The caller must stop admission.
        self._queue.put_nowait(snapshots)
        for state in snapshots:
            self._latest.pop(state["identity"], None)
            self._latest[state["identity"]] = state
        while len(self._latest) > 512:
            self._latest.popitem(last=False)

    def latest(self, identity: str, loaded: dict[str, Any] | None) -> dict[str, Any] | None:
        return self._latest.get(identity, loaded)

    @staticmethod
    def read(path: Path, identities: Iterable[str]) -> dict[str, dict[str, Any]]:
        if not path.exists():
            return {}
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
            result = {}
            for identity in identities:
                row = db.execute("SELECT state FROM objects WHERE id=?", (identity,)).fetchone()
                if row:
                    result[identity] = json.loads(row[0])
            return result

    @staticmethod
    def nearby(
        path: Path, center: NDArray[np.float64], half_extent: float
    ) -> dict[str, dict[str, Any]]:
        if not path.exists():
            return {}
        lo, hi = center - half_extent, center + half_extent
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
            rows = db.execute(
                """SELECT o.id,o.state FROM locations l JOIN objects o ON o.number=l.number
                WHERE l.minx<=? AND l.maxx>=? AND l.miny<=? AND l.maxy>=? AND l.minz<=? AND l.maxz>=?""",
                tuple(float(v) for v in (hi[0], lo[0], hi[1], lo[1], hi[2], lo[2])),
            ).fetchall()
            return {identity: json.loads(state) for identity, state in rows}

    @staticmethod
    def generation(path: Path) -> int:
        if not path.exists():
            return 0
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
            return int(db.execute("SELECT value FROM generation").fetchone()[0])

    def close(self) -> None:
        if self._thread.is_alive():
            self._queue.put(None, timeout=10)
            self._thread.join(timeout=10)
        if self._thread.is_alive():
            raise RuntimeError("Persistent state writer did not shut down")
        if self._error:
            raise RuntimeError("Persistent state writer failed") from self._error


def joint_spans(model: mujoco.MjModel) -> dict[str, tuple[int, int, int, int]]:
    result = {}
    for j in range(model.njnt):
        kind = model.jnt_type[j]
        nq, nv = (
            (7, 6)
            if kind == mujoco.mjtJoint.mjJNT_FREE
            else (4, 3)
            if kind == mujoco.mjtJoint.mjJNT_BALL  # type: ignore[attr-defined]  # Missing from local stub.
            else (1, 1)
        )
        name = model.joint(j).name
        if name in result:
            raise ValueError("Ambiguous joint identity during streaming")
        result[name] = (int(model.jnt_qposadr[j]), nq, int(model.jnt_dofadr[j]), nv)
    return result


def object_snapshot(
    model: mujoco.MjModel, data: mujoco.MjData, identity: str, body_name: str, radius: float
) -> dict[str, Any]:
    body = model.body(body_name)
    joints = {}
    spans = joint_spans(model)
    for index in range(int(body.jntadr[0]), int(body.jntadr[0] + body.jntnum[0])):
        name = model.joint(index).name
        q, nq, v, nv = spans[name]
        joints[name] = {
            "qpos": data.qpos[q : q + nq].tolist(),
            "qvel": data.qvel[v : v + nv].tolist(),
            "warmstart": data.qacc_warmstart[v : v + nv].tolist(),
        }
    return {
        "identity": identity,
        "body": body_name,
        "joints": joints,
        "position": data.xpos[body.id].tolist(),
        "quaternion_wxyz": data.xquat[body.id].tolist(),
        "radius": radius,
        "time_s": data.time,
    }


def restore_object(model: mujoco.MjModel, data: mujoco.MjData, snapshot: dict[str, Any]) -> None:
    spans = joint_spans(model)
    for name, state in snapshot["joints"].items():
        if not name.startswith(OBJECT_PREFIX) or name not in spans:
            raise ValueError("Persistent object joint identity changed")
        q, nq, v, nv = spans[name]
        if len(state["qpos"]) != nq or len(state["qvel"]) != nv or len(state["warmstart"]) != nv:
            raise ValueError("Persistent object coordinate layout changed")
        for values in state.values():
            if not np.isfinite(values).all():
                raise ValueError("Nonfinite persistent object state")
        data.qpos[q : q + nq] = state["qpos"]
        data.qvel[v : v + nv] = state["qvel"]
        data.qacc_warmstart[v : v + nv] = state["warmstart"]


def geom_equal(old: mujoco.MjModel, old_id: int, new: mujoco.MjModel, new_id: int) -> bool:
    for field in (
        "geom_type",
        "geom_pos",
        "geom_quat",
        "geom_size",
        "geom_friction",
        "geom_solref",
        "geom_solimp",
        "geom_margin",
        "geom_gap",
        "geom_contype",
        "geom_conaffinity",
        "geom_condim",
        "geom_priority",
        "geom_solmix",
    ):
        if not np.array_equal(getattr(old, field)[old_id], getattr(new, field)[new_id]):
            return False
    if old.geom_type[old_id] == mujoco.mjtGeom.mjGEOM_MESH:
        for field, address, count in (
            ("mesh_vert", "mesh_vertadr", "mesh_vertnum"),
            ("mesh_face", "mesh_faceadr", "mesh_facenum"),
        ):
            om, nm = old.geom_dataid[old_id], new.geom_dataid[new_id]
            oa, na = getattr(old, address)[om], getattr(new, address)[nm]
            on, nn = getattr(old, count)[om], getattr(new, count)[nm]
            if on != nn or not np.array_equal(
                getattr(old, field)[oa : oa + on], getattr(new, field)[na : na + nn]
            ):
                return False
    return True


def transfer_named_state(
    old: mujoco.MjModel,
    before: mujoco.MjData,
    new: mujoco.MjModel,
    after: mujoco.MjData,
    sleeping: set[str],
) -> float:
    """Preserve all shared dynamics; only explicitly sleeping environment bodies may disappear."""
    old_bodies = {old.body(i).name: i for i in range(old.nbody)}
    new_bodies = {new.body(i).name: i for i in range(new.nbody)}
    removed = old_bodies.keys() - new_bodies.keys()
    added = new_bodies.keys() - old_bodies.keys()
    if not removed <= sleeping or any(
        not name.startswith(OBJECT_PREFIX) for name in removed | added
    ):
        raise ValueError("Streaming attempted to remove an awake or non-environment body")
    for name in old_bodies.keys() & new_bodies.keys():
        oi, ni = old_bodies[name], new_bodies[name]
        if old.body(int(old.body_parentid[oi])).name != new.body(int(new.body_parentid[ni])).name:
            raise ValueError("Body parent changed during streaming")
        for field in (
            "body_mass",
            "body_inertia",
            "body_ipos",
            "body_iquat",
            "body_pos",
            "body_quat",
        ):
            if not np.array_equal(getattr(old, field)[oi], getattr(new, field)[ni]):
                raise ValueError(f"Streaming changed {field}")
        after.xfrc_applied[ni] = before.xfrc_applied[oi]
    for contact in before.contact:
        pair = [int(g) for g in contact.geom if g >= 0]
        bodies = [old.body(int(old.geom_bodyid[g])).name for g in pair]
        if any(name in removed for name in bodies):
            if any(name not in removed and old_bodies[name] != 0 for name in bodies):
                raise ValueError("Cannot unload an object touching a retained dynamic body")
            continue
        for geom in pair:
            try:
                other = new.geom(old.geom(geom).name).id
            except KeyError as failure:
                raise ValueError("Refusing to unload a contacting collider") from failure
            if not geom_equal(old, geom, new, other):
                raise ValueError("Contacting collider geometry or material changed")
    previous, incoming = joint_spans(old), joint_spans(new)
    error = 0.0
    for name in previous.keys() & incoming.keys():
        oq, nq, ov, nv = previous[name]
        iq, iqn, iv, ivn = incoming[name]
        if (nq, nv) != (iqn, ivn):
            raise ValueError("Joint coordinate type changed")
        # The imported robot root has one anonymous free joint. Its coordinate
        # address remains unambiguous; MuJoCo's name lookup rejects empty names.
        oj, nj = (
            int(np.flatnonzero(old.jnt_qposadr == oq)[0]),
            int(np.flatnonzero(new.jnt_qposadr == iq)[0]),
        )
        for field in ("jnt_type", "jnt_axis", "jnt_pos", "jnt_range", "jnt_limited"):
            if not np.array_equal(getattr(old, field)[oj], getattr(new, field)[nj]):
                raise ValueError(f"Streaming changed {field}")
        for field in ("dof_damping", "dof_armature", "dof_frictionloss"):
            if not np.array_equal(
                getattr(old, field)[ov : ov + nv], getattr(new, field)[iv : iv + nv]
            ):
                raise ValueError(f"Streaming changed {field}")
        for field, a, b, length in (
            ("qpos", oq, iq, nq),
            ("qvel", ov, iv, nv),
            ("qacc_warmstart", ov, iv, nv),
            ("qfrc_applied", ov, iv, nv),
        ):
            target = getattr(after, field)
            source = getattr(before, field)[a : a + length]
            target[b : b + length] = source
            error = max(error, float(np.max(np.abs(target[b : b + length] - source))))
    if old.nu != new.nu or old.na != new.na or old.nplugin != 0 or new.nplugin != 0:
        raise ValueError("Streamed objects may not alter actuators or plugin state")
    if any(old.actuator(i).name != new.actuator(i).name for i in range(old.nu)):
        raise ValueError("Actuator order changed")
    for field in ("ctrl", "act", "userdata", "eq_active", "mocap_pos", "mocap_quat"):
        if getattr(before, field).shape != getattr(after, field).shape:
            raise ValueError(f"Unsupported changing state layout: {field}")
        getattr(after, field)[:] = getattr(before, field)
    new.actuator_gainprm[:] = old.actuator_gainprm
    new.actuator_biasprm[:] = old.actuator_biasprm
    after.time = before.time
    if not np.isfinite(after.qpos).all() or not np.isfinite(after.qvel).all():
        raise ValueError("Nonfinite transferred state")
    mujoco.mj_forward(new, after)
    return error


def awake_objects(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    objects: dict[str, dict[str, Any]],
    settled_since: dict[str, float],
) -> set[str]:
    """Contact-connected awake bodies remain simulated, even beyond the robot window."""
    body_to_identity = {model.body(value["body"]).id: key for key, value in objects.items()}
    awake = set()
    for body, identity in body_to_identity.items():
        joint = int(model.body_jntadr[body])
        v = int(model.jnt_dofadr[joint])
        count = 6 if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE else 1
        quiet = (
            np.linalg.norm(data.qvel[v : v + count]) < 1e-3
            and np.linalg.norm(data.qacc[v : v + count]) < 0.02
            and not np.any(data.xfrc_applied[body])
            and not np.any(data.qfrc_applied[v : v + count])
        )
        if not quiet:
            settled_since.pop(identity, None)
        elif identity not in settled_since:
            settled_since[identity] = data.time
        if not quiet or data.time - settled_since[identity] < 1.0:
            awake.add(identity)
    pairs = [
        (int(model.geom_bodyid[c.geom[0]]), int(model.geom_bodyid[c.geom[1]]))
        for c in data.contact
        if min(c.geom) >= 0
    ]
    changed = True
    while changed:
        changed = False
        for a, b in pairs:
            for own, other in ((a, b), (b, a)):
                own_identity = body_to_identity.get(own)
                if own_identity is not None and own_identity not in awake and other != 0:
                    other_identity = body_to_identity.get(other)
                    if other_identity is None or other_identity in awake:
                        awake.add(own_identity)
                        changed = True
    return awake
