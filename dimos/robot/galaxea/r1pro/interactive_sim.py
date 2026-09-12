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

"""Measured object inventory and planning RPCs for the interactive house."""

from dataclasses import asdict
from typing import Any

import mujoco
import numpy as np

from dimos.core.core import rpc
from dimos.robot.galaxea.r1pro.bottle_motion import bottle_contacts, plan_unload
from dimos.robot.galaxea.r1pro.grasping_sim import BOTTLE_HALF_HEIGHT, BOTTLE_RADIUS
from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP
from dimos.robot.galaxea.r1pro.home_sim import R1ProHomeSim
from dimos.robot.galaxea.r1pro.home_surfaces import low_surface_destination, station_name
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion, TrayWaypoint
from dimos.robot.galaxea.r1pro.tray_task import TrayDestination, laptop_destination


class R1ProInteractiveSim(R1ProHomeSim):
    """Keep simulator ground truth explicit; no learned perception is claimed."""

    @rpc
    def inventory(self) -> dict[str, Any]:
        """Report bottle IDs and robot-relative spatial coordinates in metres."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            data = self._engine.data
            state = self.packing_state()
            base = data.body("base_link")
            rotation = base.xmat.reshape(3, 3)
            rows = []
            for row in state["bottles"]:
                relative = rotation.T @ (np.array(row["bottle_position"]) - base.xpos)
                rows.append(
                    {
                        **row,
                        "id": f"bottle_{row['bottle']}",
                        "forward_m": float(relative[0]),
                        "left_m": float(relative[1]),
                        "distance_m": float(np.linalg.norm(relative[:2])),
                    }
                )
            return {"bottles": rows, "tray": self.task_state()["tray"]}

    @rpc
    def set_carried_bottles(self, indices: list[int]) -> None:
        """Set cargo requirements before pickup; never infer away a dropped bottle."""
        if any(not 0 <= i < len(PACKING_BODIES) for i in indices) or len(set(indices)) != len(
            indices
        ):
            raise ValueError("Cargo indices must be unique known bottles")
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            self.cargo_bodies = tuple(PACKING_BODIES[i] for i in indices)
            self._transport_planner = None

    @rpc
    def station(self, name: str) -> dict[str, Any]:
        """Resolve a named destination against actual collidable support geometry."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            model, data = self._engine.model, self._engine.data
            name = station_name(name)
            if name in ("bed", "floor"):
                return low_surface_destination(model, data, name)
            if name == "dining_table":
                return laptop_destination(model, data).to_dict()
            if name != "kitchen":
                raise ValueError("Unknown station. Use get_surfaces to list grounded destinations.")
            # Free part of the packaged kitchen counter, beside the sink.
            point = np.array([-2.64, -1.10, 1.2])
            # A tray placed on the counter must not become the counter itself.
            excluded = {model.body(n).id for n in ("base_link", "task_bin", *PACKING_BODIES)}
            for body in range(1, model.nbody):
                if int(model.body_parentid[body]) in excluded:
                    excluded.add(body)
            supports = []
            for gid in range(model.ngeom):
                geom = model.geom(gid)
                if int(geom.bodyid[0]) in excluded:
                    continue
                if geom.type[0] != mujoco.mjtGeom.mjGEOM_BOX or not (
                    geom.contype[0] or geom.conaffinity[0]
                ):
                    continue
                rotation = data.geom_xmat[gid].reshape(3, 3)
                local = rotation.T @ (point - data.geom_xpos[gid])
                top = data.geom_xpos[gid, 2] + geom.size[2]
                if (
                    abs(rotation[2, 2]) > 0.999
                    and np.all(np.abs(local[:2]) < geom.size[:2])
                    and 0.8 < top < 0.95
                ):
                    supports.append((float(top), gid))
            if not supports:
                raise RuntimeError("Kitchen countertop was not found in this scene package")
            top, gid = max(supports)
            geom = model.geom(gid)
            target = [float(point[0]), float(point[1]), top]
            return TrayDestination(
                geom.name, target, [target[0] + 0.62, target[1], float(np.pi)]
            ).to_dict()

    @rpc
    def surfaces(self) -> list[dict[str, Any]]:
        """List measured destinations and any geometry errors before motion."""
        result = []
        for name in ("dining_table", "kitchen", "bed", "floor"):
            try:
                station = self.station(name)
                result.append(
                    {
                        "name": name,
                        "tray_position": station["tray_position"],
                        "base_position": station["base_position"],
                        "geometry_found": True,
                    }
                )
            except (RuntimeError, ValueError) as error:
                result.append({"name": name, "geometry_found": False, "reason": str(error)})
        return result

    @rpc
    def bottle_state(self, index: int) -> dict[str, Any]:
        """Read physical contact evidence while unloading one bottle."""
        if self._engine is None or not 0 <= index < len(PACKING_BODIES):
            raise ValueError("A live scene and known bottle index are required")
        with self._engine._lock:
            return bottle_contacts(self._engine.model, self._engine.data, index)

    @rpc
    def plan_bottle_unload(self, index: int, station: str) -> dict[str, Any]:
        """Choose an empty supported placement within reach, then solve grasp IK."""
        if self._engine is None or not 0 <= index < len(PACKING_BODIES):
            raise ValueError("A live scene and known bottle index are required")
        destination = self.station(station)
        with self._engine._lock:
            model = self._engine.model
            snapshot = mujoco.MjData(model)
            snapshot.qpos[:] = self._engine.data.qpos
            mujoco.mj_forward(model, snapshot)
        tray = snapshot.body("task_bin")
        rotation = snapshot.body("base_link").xmat.reshape(3, 3)
        geom = model.geom(destination["support_geom"])
        extent = np.abs(snapshot.geom_xmat[geom.id].reshape(3, 3)) @ geom.size
        low, high = snapshot.geom_xpos[geom.id] - extent, snapshot.geom_xpos[geom.id] + extent
        errors = []
        offsets = [(dx, dy) for dx in (0.17, -0.17) for dy in (-0.08, 0.0, 0.08, -0.16, 0.16)]
        offsets.extend(((0.0, -0.30), (0.0, 0.30)))
        for dx, dy in offsets:
            target = tray.xpos + rotation @ np.array([dx, dy, 0.0])
            target[2] = high[2]
            if not (
                np.all(target[:2] > low[:2] + BOTTLE_RADIUS + 0.015)
                and np.all(target[:2] < high[:2] - BOTTLE_RADIUS - 0.015)
            ):
                continue
            if any(
                np.linalg.norm(snapshot.body(n).xpos[:2] - target[:2]) < 0.08
                for i, n in enumerate(PACKING_BODIES)
                if i != index
            ):
                continue
            try:
                points = plan_unload(model, snapshot, index, target.tolist())
                return {
                    "target": target.tolist(),
                    "support_geom": geom.name,
                    "waypoints": [asdict(p) for p in points],
                }
            except RuntimeError as error:
                errors.append(str(error))
        raise RuntimeError(f"No empty reachable bottle placement on {station}: {errors}")

    @rpc
    def restore_packing_observations(self) -> None:
        """Resume RGB/goal observations after a user-requested simulation reset."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            if self._packing_biasprm is not None:
                self._engine.model.actuator_biasprm[:] = self._packing_biasprm
        self._engine.set_camera_streaming_enabled(True)
        self.cargo_bodies = ()
        self._transport_planner = None

    @rpc
    def plan_bottle_descent(self, index: int, support_geom: str) -> dict[str, Any]:
        """Plan at most 3 mm of measured descent; never open an unsupported grasp."""
        if self._engine is None or not 0 <= index < len(PACKING_BODIES):
            raise ValueError("A live scene and known bottle index are required")
        with self._engine._lock:
            model = self._engine.model
            snapshot = mujoco.MjData(model)
            snapshot.qpos[:] = self._engine.data.qpos
            snapshot.qvel[:] = self._engine.data.qvel
            snapshot.ctrl[:] = self._engine.data.ctrl
            mujoco.mj_forward(model, snapshot)
            state = bottle_contacts(model, self._engine.data, index)
        if support_geom in state["support_geoms"]:
            return {"supported": True}
        if not state["grasped"] or not state["upright"]:
            raise RuntimeError("Cannot seek support without an upright two-pad bottle grasp")
        geom = model.geom(support_geom)
        rotation = snapshot.geom_xmat[geom.id].reshape(3, 3)
        if geom.type[0] != mujoco.mjtGeom.mjGEOM_BOX or abs(rotation[2, 2]) < 0.999:
            raise RuntimeError("Bottle descent requires a horizontal supported target")
        local = rotation.T @ (np.asarray(state["position"]) - snapshot.geom_xpos[geom.id])
        if np.any(np.abs(local[:2]) > geom.size[:2] - BOTTLE_RADIUS):
            raise RuntimeError("Bottle is outside the support boundary; holding for recovery")
        motion = TrayMotion(model, snapshot, cargo_bodies=(PACKING_BODIES[index],))
        target = snapshot.site("right_tcp").xpos.copy()
        target[2] -= 0.003
        motion.right_pose(target)
        motion.probe.qpos[motion.qids[-1]] = 0.0
        point = TrayWaypoint("unload_seek_support", motion.probe.qpos[motion.qids].tolist(), 0.6)
        motion._checked([point], snapshot)
        return {"supported": False, "waypoint": asdict(point)}

    @rpc
    def plan_pick_recovery(self) -> list[dict[str, Any]]:
        """Release supported contacts and retreat to the ACT home pose after a failed pick."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            model = self._engine.model
            snapshot = mujoco.MjData(model)
            snapshot.qpos[:] = self._engine.data.qpos
            mujoco.mj_forward(model, snapshot)
            tray = self.task_state()["tray"]
            bottles = [
                bottle_contacts(model, self._engine.data, i) for i in range(len(PACKING_BODIES))
            ]
        if not tray["support_geoms"]:
            raise RuntimeError("Tray is unsupported; keep holding it or request a scene reset")
        for index, bottle in enumerate(bottles):
            if bottle["touching_pads"] and not (bottle["support_geoms"] and bottle["upright"]):
                if bottle["grasped"] and bottle["upright"]:
                    # A stopped pick may hold a bottle just above its original
                    # worktop. Seek contact in small measured steps before release.
                    table = model.body("task_table").id
                    for gid in np.flatnonzero(model.geom_bodyid == table):
                        geom = model.geom(gid)
                        rotation = snapshot.geom_xmat[gid].reshape(3, 3)
                        if geom.type[0] != mujoco.mjtGeom.mjGEOM_BOX or rotation[2, 2] < 0.999:
                            continue
                        local = rotation.T @ (
                            np.asarray(bottle["position"]) - snapshot.geom_xpos[gid]
                        )
                        bottom_gap = local[2] - BOTTLE_HALF_HEIGHT - geom.size[2]
                        if (
                            np.all(np.abs(local[:2]) < geom.size[:2] - BOTTLE_RADIUS)
                            and -0.003 <= bottom_gap <= 0.024
                        ):
                            descent = self.plan_bottle_descent(index, geom.name)
                            waypoint = descent.get(
                                "waypoint",
                                {
                                    "positions": [
                                        float(snapshot.joint(n).qpos[0])
                                        for n in R1PRO_PICK_PLACE_JOINTS
                                    ],
                                    "seconds": 0.2,
                                },
                            )
                            return [
                                {
                                    **waypoint,
                                    "phase": "recovery_seek_support",
                                    "allowed_contacts": [PACKING_BODIES[index]],
                                }
                            ]
                raise RuntimeError(f"bottle_{index + 1} is unsupported; keeping the gripper closed")
        home = self.config.reset_joint_positions
        if home is None or len(home) < len(R1PRO_PICK_PLACE_JOINTS):
            raise RuntimeError("No calibrated ACT home posture is configured")
        errors = []
        for clearance in (1.04, 0.98, 0.94):
            # A high retreat can swing the elbow into furniture. Try a finite
            # set of lower routes, checking each complete sweep before moving.
            motion = TrayMotion(model, snapshot, cargo_bodies=PACKING_BODIES)
            motion.probe.qpos[motion.qids[-2:]] = 0.05
            points = [
                TrayWaypoint("recovery_release", motion.probe.qpos[motion.qids].tolist(), 0.8)
            ]
            above = snapshot.site("right_tcp").xpos.copy()
            above[2] = max(above[2], clearance)
            try:
                for phase, target in (
                    ("recovery_lift", above),
                    ("recovery_home", np.asarray(HOME_TCP)),
                ):
                    motion.right_pose(target)
                    points.append(TrayWaypoint(phase, motion.probe.qpos[motion.qids].tolist(), 2.0))
                points.append(TrayWaypoint("recovery_posture", list(home[: len(motion.qids)]), 3.0))
                result = [asdict(p) for p in motion._checked(points, snapshot)]
                break
            except RuntimeError as error:
                errors.append(str(error))
        else:
            raise RuntimeError("No clear recovery retreat: " + "; ".join(errors))
        # Opening an existing supported grasp necessarily starts in contact.
        # Permit only those bottles, and only during that release waypoint.
        result[0]["allowed_contacts"] = [
            PACKING_BODIES[i] for i, bottle in enumerate(bottles) if bottle["touching_pads"]
        ]
        return result
