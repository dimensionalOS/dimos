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

"""What the viewer asks the memory world for: questions, routes and orbit frames.

A mixin for :class:`MemoryWorldModule` (``module.py`` is at the repository's file-size
limit). The answer itself is built in :mod:`visual_answers` from the recording's own
CLIP/SigLIP frame embeddings; this file owns what surrounds it -- the HTTP routes the
page calls, the route planner behind **Navigate**, and the tf frames the orbit picker
offers.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

from fastapi import HTTPException
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
import numpy as np
from pydantic import BaseModel, Field, field_validator

from dimos.agents.skill_result import SkillResult
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.teleop.memory_world.messages import encode_text
from dimos.teleop.memory_world.query import HighlightPath, MemoryQueryResult
from dimos.teleop.memory_world.replay import frame_positions
from dimos.teleop.memory_world.route import (
    MLS_MAX_VOXELS,
    MlsRoutePlanner,
    RoutePlanner,
    mls_available,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A storey. Used to decide which floor the viewer is standing on: they cannot be standing
# on one above their own head, and the one they are on is within a storey below it.
STOREY_M = 3.0


def _as_json_can_hold_it(value: float) -> float | str:
    """*value*, or its name when json has no way to write it (`nan`, `inf`, `-inf`)."""
    return value if math.isfinite(value) else repr(value)


# How many distinct early poses of the robot's path are offered as the start of a route
# "from the starting point". Each one is a full planning attempt, so this bounds the work;
# a recording where two dozen consecutive early poses can all reach nothing has something
# wrong with it that a longer search would hide rather than fix.
RECORDING_START_TRIES = 24


class AskRequest(BaseModel):
    text: str = Field(min_length=1, max_length=400)
    # Which stretch of the recording to look in, as fractions of its length: 0 is the
    # beginning and 1 the end, so "the first half" is 0.0 to 0.5. The same restriction
    # the agent's `find_in_memory` tool takes, so the viewer demonstrates the lookup the
    # LLM actually performs rather than a second one that only looks similar.
    from_fraction: float = Field(default=0.0, ge=0.0, le=1.0)
    to_fraction: float = Field(default=1.0, ge=0.0, le=1.0)


class NavigateRequest(BaseModel):
    query_id: str | None = None  # the answer the cluster belongs to, when the viewer knows it
    cluster: int = Field(default=0, ge=0)
    # Start from here (world xyz) instead of under the viewer.
    start: tuple[float, float, float] | None = None
    # Or start from a place the RECORDING defines rather than one the viewer sends.
    # "recording_start" is where the robot was when the recording began -- what someone
    # means by "from the starting point", which is not where they happen to be standing.
    # It wins over `start`: a caller that names both has said the more specific thing.
    start_at: Literal["viewer", "recording_start"] = "viewer"
    # Walk to the PHOTO the viewer has stepped to, not to the middle of the blob. The
    # index is the one the query-image header carries, so the viewer names a picture it
    # was actually sent rather than posting a position of its own.
    view: int | None = None

    @field_validator("start")
    @classmethod
    def _a_place_a_body_could_be(
        cls, where: tuple[float, float, float] | None
    ) -> tuple[float, float, float] | None:
        # pydantic takes `NaN` and `inf` as floats, and a NaN start reached
        # `RoutePlanner.cell_of`, whose `math.floor` raises
        # `ValueError: cannot convert float NaN to integer` -- not an `HTTPException`, so
        # the endpoint answered 500 "Internal Server Error" where every other bad route
        # answers 404, 409, 422 or 503 with a sentence. The websocket's `viewer_pose` has
        # checked this since two exceptions killed its loop, and `pose_of` below checks it
        # on the photo headers; this was the one position from a client that did not.
        if where is not None and not all(math.isfinite(value) for value in where):
            raise ValueError("start must be a finite xyz")
        return where


class WorldAnswers:
    """The ask/navigate/orbit half of the module. Expects the host module's
    ``config``, ``_ensure_store``, ``_broadcast``, ``_publish_query_result``,
    ``_clients_lock``, ``_active_query_images``, ``_tf_tree``, ``_frame_pose_at``,
    ``_replay_index_json`` and ``_cached_cloud``."""

    # The host's attributes and methods this mixin relies on.
    config: Any
    _clients_lock: threading.Lock
    _active_query_images: list[tuple[dict[str, Any], bytes]]
    _cached_cloud: tuple[dict[str, Any], bytes] | None
    _world_clients: Any
    _map_xyz: np.ndarray | None  # the whole map, before the viewer's stride
    _viewer_position: tuple[float, float, float] | None
    _store_lock: Any  # an RLock

    if TYPE_CHECKING:

        def _broadcast(self, message: bytes | str) -> None: ...
        def _publish_query_result(self, result: MemoryQueryResult) -> str: ...
        def _ensure_store(self) -> Any: ...
        def _camera_frame(self) -> str: ...
        def _tf_tree(self) -> Any: ...
        def _effective_orbit_frame(self) -> str: ...
        def _frame_pose_at(self, frame: str, ts: float) -> Any: ...
        def _replay_index_json(self) -> dict[str, Any]: ...
        def _index_status(self) -> dict[str, Any]: ...
        def find_in_memory(
            self, query: str, from_fraction: float = 0.0, to_fraction: float = 1.0
        ) -> SkillResult: ...

    def _init_answers(self) -> None:
        # The places on screen and the query id they were published under. `/navigate`
        # and `/answer` both read it; `visual_answers` writes it.
        self._last_answer: tuple[Any | None, str | None] = (None, None)
        self._route_planner: RoutePlanner | MlsRoutePlanner | None = None
        # First in the lock order: the planner reads the orbit, which reads the replay.
        self._planner_lock = threading.RLock()
        self._orbit_cache: dict[str, dict[str, Any]] = {}

    # ---- the map the planner walks over ------------------------------------

    def _map_points(self) -> np.ndarray | None:
        """The ray-traced map's voxel centres, when the world cache is built: the whole
        map, not the stride-sampled payload the viewer gets."""
        if self._map_xyz is not None:
            return self._map_xyz
        cached = self._cached_cloud  # read once: a reopen clears it
        if cached is None:
            return None
        header, payload = cached
        n = int(header.get("n", 0))
        return np.frombuffer(payload, dtype=np.float32, count=n * 3).reshape(n, 3)

    def _join_world_clients(self, conn: Any) -> None:
        """Register a viewer, and start it on a clean world if the last session ended.

        `_active_query_result` and its evidence photos are replayed to every websocket
        that connects, which is what lets a second viewer -- a headset picked up beside
        a laptop -- join a demo already in progress. Nothing ever cleared them, so they
        were not session state at all: they lived as long as the process, and the next
        person to open the page was greeted by markers and photographs answering a
        question they never asked. Every path that answers wrote them -- a curl, an
        agent turn, another tab, the tour, and on the hyperspace build any answer
        published by anybody -- so this had many ways to happen and no way not to.

        An answer belongs to the viewing session that asked for it. A viewer arriving
        to an EMPTY set is the start of a new session and gets a clean world; one
        arriving while others are watching is joining theirs and still inherits it.
        Caller holds `_clients_lock`.
        """
        if not self._world_clients:
            self._forget_the_answer()
        self._world_clients.add(conn)

    def _forget_the_answer(self) -> None:
        """Drop the answer on screen and its evidence. Caller holds `_clients_lock`.

        All three together: the result the wire replays, the photographs behind it, and
        the places `/navigate` plans from. Leaving any one behind leaves the viewer able
        to route to a marker it is no longer showing.
        """
        self._active_query_result = None
        self._active_query_images = []
        self._last_answer = (None, None)

    def _query_is_current(self, query_id: str) -> bool:
        current = getattr(self, "_active_query_result", None)
        return isinstance(current, dict) and current.get("query_id") == query_id

    # ---- navigation --------------------------------------------------------

    def _planner(self) -> RoutePlanner | MlsRoutePlanner:
        """The MLS 3D planner over the map, built once; the 2D costmap when the
        binding is missing or the map is a city."""
        with self._planner_lock:
            if self._route_planner is not None:
                return self._route_planner
            points = self._map_points()
            if points is None:
                raise HTTPException(status_code=503, detail="the map is still building")
            voxels = points.astype(np.float64)
            voxel_size = float(self.config.voxel_size)
            if mls_available() and len(voxels) <= MLS_MAX_VOXELS:
                started = time.monotonic()
                mls = MlsRoutePlanner(voxels, voxel_size=voxel_size)
                logger.info(
                    "MLS planner: %d surface cells from %d voxels in %.1f s",
                    mls.surface_cells,
                    len(voxels),
                    time.monotonic() - started,
                )
                self._route_planner = mls
            else:
                logger.info(
                    "costmap planner (mls binding %s, %d voxels)",
                    "present" if mls_available() else "missing",
                    len(voxels),
                )
                known = self._orbit_positions_for(self._effective_orbit_frame()).get("positions")
                if not known:
                    # The costmap's free corridor IS the robot's path. With no path there
                    # is nothing to plan over, and a 500 out of RoutePlanner says less than
                    # this does. An explicit start reaches here too, hence the check.
                    raise HTTPException(status_code=503, detail="the robot's path is not known yet")
                path = np.asarray(known, dtype=np.float64).reshape(-1, 3)
                self._route_planner = RoutePlanner.from_voxels(voxels, path, voxel_size=voxel_size)
            return self._route_planner

    def _robot_end_pose(self) -> tuple[float, float, float]:
        """Where the robot's orbit frame was at the end of the recording."""
        positions = self._orbit_positions_for(self._effective_orbit_frame()).get("positions") or []
        if positions:
            return tuple(float(v) for v in positions[-1])  # type: ignore[return-value]
        raise HTTPException(status_code=503, detail="the robot's path is not known yet")

    def _recording_start_candidates(self) -> list[tuple[float, float, float]]:
        """Where a route "from the starting point" may begin, best first.

        The recording's first pose, then successively later ones along the robot's own
        path. More than one is needed because the first is often not a place a route can
        LEAVE: the map is ray-traced and a voxel is kept only once several scans agree on
        it, so at t=0 there is barely any map around the robot. Measured on grocery.mcap,
        `/navigate` refused from the first five distinct poses and planned 25.68 m from
        the twentieth -- about 2.7 seconds in.

        Every one of them is a place the robot demonstrably was, so the answer is still
        "the starting point"; the payload names the one that worked.

        Being able to STAND somewhere is not the same as being able to leave it, which is
        why this returns candidates for the planner to try rather than picking one by
        asking `candidates()`: a pose can have standable cells near it and still sit on an
        island the map never connected. That mistake made the first version of this fix
        return the very first pose and refuse exactly as before.
        """
        positions = self._orbit_positions_for(self._effective_orbit_frame()).get("positions") or []
        if not positions:
            raise HTTPException(status_code=503, detail="the robot's path is not known yet")
        starts: list[tuple[float, float, float]] = []
        previous: tuple[float, float, float] | None = None
        for sample in positions:
            where = (float(sample[0]), float(sample[1]), float(sample[2]))
            # `frame_positions` repeats a position through gaps and back-fills the
            # beginning with the first known one, so the head of this list is the same
            # point many times over. Offering it repeatedly would spend the whole budget
            # on one pose.
            if where == previous:
                continue
            previous = where
            # Through the same floor snap an explicit `start` gets, because that is the
            # path this was proved on; the raw pose stands in when the snap declines.
            starts.append(self._ground_under_viewer(where) or where)
            if len(starts) >= RECORDING_START_TRIES:
                break
        return starts

    def _ground_under_viewer(
        self, viewer: tuple[float, float, float] | None = None
    ) -> tuple[float, float, float] | None:
        """Where the viewer is standing, at a height the planner can actually use.

        *viewer* is the position to place; the shared `_viewer_position` when none is
        given. A caller's own position needs this treatment just as much as the shared
        field does -- it is the same camera height from the same `getViewerRobotPosition`
        -- and taking it raw put the start a metre above every cell the planner can stand
        on, which is the failure the rest of this docstring is about.

        A route has to start where the person asking for it is standing. It used to start
        where the ROBOT stopped at the end of the recording, so walking anywhere and
        pressing Navigate drew a green tube that began somewhere else entirely.

        The viewer's x and y are kept exactly. Their z is not: it is a CAMERA height, not
        a floor, and the client only started sending a real one recently -- for most of
        this package's history `getViewerRobotPosition()` returned `[x, y, 0]`, a literal
        zero. So z is used only to decide WHICH FLOOR, never as the answer, and a zero
        from an old client has to keep working.

        The height comes from the nearest sample of the robot's own path, and that is the
        point of this function. Two earlier versions took it from the map voxels in a
        column around the viewer and both were wrong in a way only the live demo showed:
        the highest voxel at or below `viewer[2]` is a test against zero, which on a floor
        at 0.7 matched nothing; and the LOWEST voxel in the column is a stray return under
        the floor, measured at z=-1.24 and -1.56 on the office recording. A start at an
        impossible height is worse than a wrong one -- the planner snapped it to some far
        corner of its graph and returned the SAME 47-point route for two viewers 2 m
        apart, beginning 4 m from either of them, while the payload's `start` field went
        on claiming the viewer's position.

        The robot drove its path, so every height along it is one the planner can stand
        at. Restrict to the samples in the storey-deep band below the camera, then take
        the nearest of those in x and y and use its z.

        That band is not the same thing as the viewer's floor, and on a map with two
        levels less than a storey apart it can hold both -- a viewer on a platform at 1.4
        with their camera at 3.0 gets the 0.3 floor underneath it if that is nearer in x
        and y. Separating those two needs a prior on how tall the person is, which is
        exactly the guess this function exists to avoid; a storey-deep band is the widest
        rule that needs no such guess. Single-level maps, which is every map this demo
        runs on, are unaffected.
        """
        if viewer is None:
            with self._clients_lock:
                viewer = self._viewer_position
        if viewer is None:
            return None
        known = self._orbit_positions_for(self._effective_orbit_frame()).get("positions")
        if not known:
            return None
        path = np.asarray(known, dtype=np.float64).reshape(-1, 3)
        if not len(path):
            return None
        # Which floor, then which point on it. Nearest in the plane alone puts a viewer
        # standing on a mezzanine three metres below their own feet; nearest in 3-D is
        # worse, and worse on the maps we actually have -- the gap between a camera and
        # the sensor that drove the path is metres, so on a SINGLE-storey map a sample
        # upstairs can be nearer the viewer's eyes than the floor they are standing on
        # (measured: samples at z=0.3 and z=2.8, a ground-floor viewer at 1.7, and 2.8
        # wins).
        #
        # The signal that does not need a guess about human height: you cannot be standing
        # on a floor above your own head, and the one you are on is within a storey below
        # it. Restrict to those, then take the nearest in the plane. Nothing in the band
        # (an old client sending z=0, a viewer flying) falls back to the plane over every
        # sample, which is the behaviour before any of this.
        flat = (path[:, 0] - viewer[0]) ** 2 + (path[:, 1] - viewer[1]) ** 2
        on_this_floor = (path[:, 2] <= viewer[2]) & (path[:, 2] >= viewer[2] - STOREY_M)
        usable = np.flatnonzero(on_this_floor)
        nearest = int(usable[np.argmin(flat[usable])]) if len(usable) else int(np.argmin(flat))
        return (float(viewer[0]), float(viewer[1]), float(path[nearest, 2]))

    def _navigate_to(self, request: NavigateRequest) -> dict[str, Any]:
        with self._clients_lock:
            answer, query_id = self._last_answer
            active = getattr(self, "_active_query_result", None)
        if not isinstance(active, dict) or active.get("query_id") != query_id:
            raise HTTPException(status_code=409, detail="there is no answer on screen to route to")
        if request.query_id not in (None, query_id):
            raise HTTPException(status_code=409, detail="that answer has been replaced")
        if answer is None or request.cluster >= len(answer.clusters):
            raise HTTPException(status_code=404, detail="no such cluster in the last answer")
        cluster = answer.clusters[request.cluster]
        # The caller's own position first: `_viewer_position` is ONE field for the whole
        # module, written by whichever client last sent a pose, so with two people in the
        # same world it is the other one's feet as often as not -- and the route was drawn
        # from there and reported as fact. `results.js` sends `start` with every Navigate
        # now. The shared field stays as the fallback for a caller that sends none (a
        # script, or a viewer whose render loop has not run yet), which is exactly the
        # case where there is only one viewer to confuse.
        # Through the floor snap either way. The caller's own position is the same camera
        # height the shared field holds -- `results.js` sends exactly
        # `getViewerRobotPosition()` -- so taking it raw started the route a metre above
        # every cell the planner can stand on, which is what this snap exists to prevent.
        if request.start_at == "recording_start":
            # A LIST, not a point: the recording's first pose is often somewhere a route
            # cannot leave, so the planner is offered successively later poses of the
            # robot's own path until one works.
            start_options = self._recording_start_candidates()
        else:
            under = (
                self._ground_under_viewer(tuple(request.start))
                if request.start
                else self._ground_under_viewer()
            )
            # And no fallback to the caller's raw position when the snap DECLINES. It
            # declines when the robot's path is not known, which is the one state in which
            # nothing can say what height the caller is standing at -- taking their camera
            # height then is the very input the snap exists to refuse. `_robot_end_pose()`
            # answers the same way it does for a caller who sent no start at all: 503.
            start_options = [under or self._robot_end_pose()]
        # The first option is what the rest of this method measures distances from; the
        # one that actually planned replaces it below, and is what the payload reports.
        start = start_options[0]
        with self._clients_lock:
            images = list(self._active_query_images)

        # By the header's OWN `index`, not by position in this list. A frame that cannot
        # be decoded is skipped when the images are published, so the list is compacted
        # while the headers keep their original numbers -- and the viewer sends the
        # number, because that is what it was given. One unreadable frame therefore
        # shifted every later photograph: asking for view 1 of 2, with view 0 unreadable,
        # got "404 no such view in the last answer" for a picture that was on screen.
        by_index = {int(header.get("index", slot)): slot for slot, (header, _) in enumerate(images)}

        def pose_of(index: int) -> tuple[float, float, float] | None:
            slot = by_index.get(index)
            if slot is None:
                return None
            header = images[slot][0]
            if header.get("query_id") != query_id or header.get("cluster") != cluster.index:
                return None
            try:
                where = tuple(float(v) for v in (header.get("position") or []))
            except (TypeError, ValueError):
                return None
            if len(where) != 3 or not all(math.isfinite(v) for v in where):
                return None
            return where  # type: ignore[return-value]

        asked: tuple[float, float, float] | None = None
        if request.view is not None:
            if request.view not in by_index:
                raise HTTPException(status_code=404, detail="no such view in the last answer")
            asked = pose_of(request.view)
            if asked is None:
                raise HTTPException(status_code=409, detail="that view is not in this place")

        # The candidates, in order of what the person asked for. The centre of a cluster
        # is a weighted mean of its voxels, so for a thing on a wall it is INSIDE the
        # wall: neither where the viewer is looking nor anywhere a body can stand. A
        # photo's camera pose is somewhere the robot already stood.
        #
        # Every one of them is tried, and that is the point. Measured on the office
        # recording from one standing position: the centre was unreachable, only 2 of
        # cluster 0's 12 views could be routed to, and the FIRST photo of all 12 places
        # was unreachable -- so Navigate answered "no route" for every place on screen
        # while a 3.48 m route to the second photo of the first place existed the whole
        # time. The closest photo the robot can still reach is a better answer than
        # refusing, and saying WHICH one it picked keeps it honest.
        # Nearest first. `images` is in publication (score) order, and taking the first
        # routable one in THAT order hands back a photo 10 m away when a reachable one
        # 1 m away was in the same list -- which is not what the paragraph above promises
        # and not what someone pressing Navigate wants to walk.
        others = sorted(
            (i for i in by_index if i != request.view and pose_of(i) is not None),
            key=lambda i: math.dist(start, pose_of(i)),  # type: ignore[arg-type]
        )
        candidates: list[tuple[int | None, tuple[float, float, float]]] = []
        if request.view is not None and asked is not None:
            candidates.append((request.view, asked))
        candidates.append((None, tuple(cluster.centre)))
        candidates.extend((i, pose) for i in others if (pose := pose_of(i)) is not None)

        planner = self._planner()
        # Everything the payload needs is captured HERE, not read off the loop variable
        # afterwards. `route` does survive the loop correctly today, because the only way
        # out with a goal is the break -- but a payload that reads a name the loop last
        # happened to leave behind is one edit away from reporting a rejected candidate's
        # length beside the accepted candidate's points.
        goal, goal_view, points, taken = None, None, [], None
        # Which start it was planned from, so the payload cannot claim one the route does
        # not begin at -- the same rule the goal has followed since the loop below learned
        # to try more than one candidate.
        used_start = start
        # Every candidate, with no cap. A cap of 8 was arbitrary and did the very thing
        # this loop exists to stop: with twelve photos of a place and only the ninth
        # reachable, it refused while a 9 m route existed. The list is already bounded --
        # it is one place's photographs, not the whole answer's.
        for attempt in start_options:
            for view_index, candidate in candidates:
                route = (
                    planner.plan(tuple(attempt), tuple(candidate))
                    if isinstance(planner, MlsRoutePlanner)
                    else planner.plan(attempt[:2], candidate[:2])
                )
                if route is None:
                    continue
                found = [(float(x), float(y), float(z)) for x, y, z in route.points]
                if len(found) < 2:
                    continue
                if math.dist(found[0], found[-1]) <= 1e-9:
                    continue  # went nowhere; see below
                goal, goal_view, points, taken = candidate, view_index, found, route
                used_start = attempt
                break
            if goal is not None:
                break
        if goal is None or taken is None:
            # Name what was tried. "No route" over a start the caller did not choose and
            # cannot see is unactionable -- and when `start_at=recording_start` walks the
            # robot's early path, WHICH poses it walked is the whole of the diagnosis.
            tried = ", ".join(
                "(" + ", ".join(f"{v:.2f}" for v in option) + ")" for option in start_options[:3]
            )
            raise HTTPException(
                status_code=422,
                detail=(
                    f"no route through the known free space: {len(candidates)} goal(s) from "
                    f"{len(start_options)} start(s) [{tried}...]"
                ),
            )
        # The "went nowhere" test in the loop above: a route has to GO somewhere. The
        # planner can return a handful of identical points when it cannot connect the
        # start to the goal, and a length check alone passes them -- measured live at
        # three copies of (2.95, 2.15, 0.7), length 0.0, returned as HTTP 200 with the
        # goal 2.45 m away, so the viewer drew a tube of no length and the person was
        # told a route existed.
        #
        # It tests the route's own EXTENT, not its progress toward the goal, because
        # extent is what actually failed. Comparing distances to the goal imports that
        # goal's height into the verdict: a real 0.5 m route down a 15 cm step, to a
        # camera pose 1.8 m up, closed 0.38 m of horizontal gap and was still refused,
        # since 3-D it read 1.856 -> 1.951. The plane fixes that particular case and
        # still asks a question the planner was never posed -- in the costmap branch z is
        # not even an input. Zero extent is unambiguous and needs nothing but the points.
        payload = {
            "query_id": query_id,
            "cluster": cluster.index,
            "start": [float(v) for v in used_start],
            # So the viewer can MARK a start the person did not choose by standing there.
            # A route drawn from the recording's beginning with no marker on it reads as a
            # route from wherever the viewer happens to be.
            "start_at": request.start_at,
            "goal": [float(v) for v in goal],
            "view": goal_view,  # which picture, when the route is to one
            "length_m": round(taken.length_m, 2),
            "points": [[round(v, 3) for v in p] for p in points],
            "cells": taken.cells,
            "planner": taken.planner,
        }
        with self._clients_lock:  # still the answer on screen? then reconnects get the route too
            if self._active_query_result is active:
                active["route"] = HighlightPath(
                    points=points, label=f"Route to #{cluster.index + 1}", color="#64ff8f"
                ).model_dump(mode="json")
            else:
                raise HTTPException(status_code=409, detail="the answer changed while planning")
        self._broadcast(encode_text("route", **payload))
        return payload

    # ---- the recording's own costmap (Go2 recordings carry one) -------------

    def _add_route_to_result(self, result: MemoryQueryResult) -> None:
        # Routes are server-owned: only the planner may label one collision-aware.
        result.route = None
        with self._clients_lock:
            viewer_position = self._viewer_position
        if result.focus_point is None or viewer_position is None:
            return
        try:
            with self._store_lock:
                store = self._ensure_store()
                if "global_costmap" not in store.list_streams():
                    return
                costmap = store.streams.global_costmap.last().data
            route = min_cost_astar(
                costmap,
                goal=result.focus_point[:2],
                start=viewer_position[:2],
            )
            if route is None:
                return
            # Half a cell, for the same reason RoutePlanner.plan adds it: min_cost_astar
            # returns OccupancyGrid.grid_to_world, which is `origin + cell * resolution`
            # -- the cell's CORNER. Without it every waypoint of a route over the
            # recording's own costmap sits down and left of the cell it was planned
            # through. The round-41 fix only reached the other caller.
            half = costmap.resolution / 2
            points = [(pose.x + half, pose.y + half, pose.z + 0.08) for pose in route.poses]
            # Two points that are the same point are not a route. `_navigate_to` refuses
            # exactly this, with a comment recording it measured live -- three copies of
            # one pose returned as a successful 200 -- and this caller, which runs
            # automatically on every analysis answer carrying a focus point, only counted
            # them. A zero-length line drawn on the map says "here is the way there"
            # about somewhere the viewer is already standing.
            if len(points) >= 2 and math.dist(points[0], points[-1]) > 1e-9:
                result.route = HighlightPath(
                    points=points,
                    label="Route to answer",
                    color="#64ff8f",
                )
        except Exception:
            logger.exception("failed to build route to memory query result")

    # ---- tf frames for the orbit picker -----------------------------------

    def _tf_frames(self) -> list[str]:
        tree = self._tf_tree()
        return sorted(tree.frames) if tree is not None else []

    def _orbit_positions_for(self, frame: str) -> dict[str, Any]:
        """Where *frame* was at each replay scan (cached per frame)."""
        with self._planner_lock:  # a reopen clears the cache under it
            if frame in self._orbit_cache:
                return self._orbit_cache[frame]
            try:
                index = self._replay_index_json()
            except Exception as error:  # building, or failed: 503, like the replay routes
                raise HTTPException(
                    status_code=503, detail=f"replay {self._replay_progress}"
                ) from error
            if frame == index.get("orbit", {}).get("frame"):
                self._orbit_cache[frame] = index["orbit"]
                return index["orbit"]
            tree = self._tf_tree()
            if tree is None or not tree.has_frame(frame):
                raise HTTPException(status_code=404, detail=f"no tf frame {frame!r}")
            stamps = np.asarray(
                index.get("scans") or [], dtype=np.float64
            )  # one stamp per replay scan
            positions = frame_positions(stamps, lambda ts: self._frame_pose_at(frame, ts))
            result = {"frame": frame, "positions": positions}
            self._orbit_cache[frame] = result
            return result

    # ---- routes ------------------------------------------------------------

    def _ask_the_agent(self, text: str) -> dict[str, Any] | None:
        """Put the whole sentence to the LLM agent; return its turn, or None if it has none.

        The agent is the difference between a question and an ORDER. `find_in_memory` is a
        similarity lookup and embeds whatever string it is given, so "navigate to the first
        basket in the recording" searches for that entire sentence -- the demo Jeff saw,
        which reported six places and walked nowhere. The agent reads it, calls
        `find_in_memory` with just "a basket", reads `seconds_into_recording` to find which
        one was FIRST, and then calls `navigate_to_place` because it was told to navigate.

        Returning None means "no agent answered", and the caller falls back to the skill.
        That is deliberate: this ships in a blueprint that may be run without an agent, and
        a demo that hangs is worse than one that answers the old way.
        """
        # Imported here, not at module scope: `memory-world-module` runs this file with no
        # agent anywhere, and the transport stack should not be a hard import for it.
        from dimos.core.transport_factory import make_transport

        replies: list[str] = []
        turn_started = threading.Event()
        turn_done = threading.Event()

        def on_agent(message: Any) -> None:
            # Only the assistant's own messages; a ToolMessage is the tool's return value.
            if type(message).__name__ not in ("AIMessage", "AIMessageChunk"):
                return
            content = getattr(message, "content", None)
            # `content` is a plain string on some models and a LIST OF BLOCKS on others --
            # measured, gpt-5.6-luna answers with
            # `[{'type': 'text', 'text': 'A route was drawn...'}]`, alongside `reasoning`
            # and `function_call` blocks that are not the answer. Reading only the string
            # form dropped every reply this agent has ever sent and fell back to the skill
            # on a turn that had done exactly the right thing.
            if isinstance(content, str):
                text = content
            elif isinstance(content, list):
                text = " ".join(
                    block["text"]
                    for block in content
                    if isinstance(block, dict)
                    and block.get("type") == "text"
                    and isinstance(block.get("text"), str)
                )
            else:
                text = ""
            if text.strip():
                replies.append(text.strip())

        def on_idle(flag: Any) -> None:
            # False at the start of a turn, True at its end. A turn that never starts
            # (nothing is listening) leaves both unset and we time out into the fallback.
            if flag is False:
                turn_started.set()
            elif flag is True and turn_started.is_set():
                turn_done.set()

        transports = []
        try:
            reply_t = make_transport(self.config.agent_reply_topic)
            idle_t = make_transport(self.config.agent_idle_topic)
            human_t = make_transport(self.config.agent_input_topic)
            transports = [reply_t, idle_t, human_t]
            for transport in transports:
                transport.start()
            reply_t.subscribe(on_agent)
            idle_t.subscribe(on_idle)
            human_t.publish(text)
            finished = turn_done.wait(self.config.agent_timeout_s)
        except Exception:
            logger.exception("could not reach the agent; answering with the skill instead")
            return None
        finally:
            for transport in transports:
                try:
                    transport.stop()
                except Exception:  # a transport that never started has nothing to close
                    logger.debug("agent transport did not stop cleanly", exc_info=True)

        if not finished or not replies:
            # Said apart, because they mean different things: a turn that never finished is
            # a missing or wedged agent, while a finished turn with nothing to say is this
            # code failing to read the reply -- which is what a content-block answer did.
            logger.warning(
                "%s; answering with the skill instead (started=%s, replies=%d)",
                (
                    f"no agent turn finished within {self.config.agent_timeout_s:.0f}s"
                    if not finished
                    else "the agent turn finished but carried no readable reply"
                ),
                turn_started.is_set(),
                len(replies),
            )
            return None
        # The LAST assistant message is the answer; the earlier ones are its narration
        # between tool calls. The world was already lit by the tool calls themselves.
        return {"success": True, "answer": replies[-1], "metadata": {"engine": "agent"}}

    def _setup_answer_routes(self, app: Any) -> None:
        base = self.config.client_route

        @app.exception_handler(RequestValidationError)  # type: ignore[misc]
        async def _a_refusal_the_client_can_actually_read(
            request: Any, refused: RequestValidationError
        ) -> JSONResponse:
            """FastAPI's own handler cannot render a refusal about `NaN`.

            `errors()` quotes the offending value back under `input`, and Starlette
            renders the body with `json.dumps(..., allow_nan=False)`, which raises
            `ValueError: Out of range float values are not JSON compliant` before any
            body exists. So `POST /navigate {"start": [NaN, 0, 0]}` -- the very input the
            validator on `start` was written to refuse with a sentence -- came back as a
            bare 500 with no body, as did `{"cluster": NaN}` and `POST /ask {"text":
            NaN}`, while `{"cluster": -1}` answered a clean 422. The number that cannot
            be written is written as its name.
            """
            return JSONResponse(
                status_code=422,
                content={
                    "detail": jsonable_encoder(
                        refused.errors(), custom_encoder={float: _as_json_can_hold_it}
                    )
                },
            )

        @app.post(f"{base}/ask")  # type: ignore[misc]
        async def memory_world_ask(request: AskRequest) -> dict[str, Any]:
            """A typed question: same path as a spoken one."""
            self._broadcast(encode_text("voice_transcript", text=request.text))
            # The whole sentence goes to the agent, which decides what it is being asked
            # for. Only when there is no agent to ask -- or it did not answer -- does the
            # sentence go to the similarity lookup as a search phrase.
            tried_agent = (
                self.config.ask_via_agent
                and request.from_fraction == 0.0
                and request.to_fraction == 1.0
            )
            if tried_agent:
                answered = await asyncio.to_thread(self._ask_the_agent, request.text)
                if answered is not None:
                    return answered
            outcome = await asyncio.to_thread(
                self.find_in_memory, request.text, request.from_fraction, request.to_fraction
            )
            message = outcome.message
            if tried_agent:
                # Say WHICH thing failed. On the hyperspace blueprint the recording has no
                # siglip index at all, so this fallback can only ever answer "build the
                # index" -- advice about a component the question never involved, handed to
                # someone whose real problem is that the agent did not answer.
                message = f"The agent did not answer in time. Falling back: {message}"
            return {
                "success": outcome.success,
                "answer": message,
                "metadata": outcome.metadata,
            }

        @app.get(f"{base}/frames")  # type: ignore[misc]
        async def memory_world_frames() -> dict[str, Any]:
            frames = await asyncio.to_thread(self._tf_frames)
            # The configured frame only if tf has it: /orbit falls back to the camera,
            # and offering a default nothing matches lets the browser pick its own.
            chosen = self.config.orbit_frame
            if frames and chosen not in frames:
                # to_thread, like every other route here: _camera_frame takes the store
                # lock and reads the store on a cold cache, and the cache's only warmer
                # runs INSIDE that lock. On the event loop it freezes every websocket and
                # every route for every client, not just this request. Called once, too.
                camera = await asyncio.to_thread(self._camera_frame)
                chosen = camera if camera in frames else frames[0]
            return {"frames": frames, "default": chosen}

        @app.get(f"{base}/orbit")  # type: ignore[misc]
        async def memory_world_orbit(frame: str) -> dict[str, Any]:
            return await asyncio.to_thread(self._orbit_positions_for, frame)

        @app.post(f"{base}/navigate")  # type: ignore[misc]
        async def memory_world_navigate(request: NavigateRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self._navigate_to, request)

        @app.get(f"{base}/answer")  # type: ignore[misc]
        async def memory_world_answer() -> dict[str, Any]:
            """The last answer's places and stats, for scripts and the tour."""
            answer, _ = self._last_answer
            if answer is None:
                return {"text": None, "clusters": []}
            stats = getattr(answer, "stats", None) or {}
            return {
                "text": getattr(answer, "text", None),
                "frame": getattr(answer, "frame", None),
                "clusters": [c.model_dump(mode="json") for c in answer.clusters],
                "stats": {k: v for k, v in stats.items() if not isinstance(v, dict | list)},
                "seconds": round(float(getattr(answer, "seconds", 0.0)), 3),
            }
