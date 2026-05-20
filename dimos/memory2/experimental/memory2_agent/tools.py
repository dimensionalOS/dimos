# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""LangChain tool wrappers around dimos.memory2 queries.

Tools accept a `stream` name so the agent can pick which logical store
to query. Streams come straight from the recorded .db passed in via
--db (no fabricated data):

    color_image            — ego-centric RGB frames
    color_image_embedded   — CLIP-embedded frames (semantic search)
    lidar                  — point clouds
    odom                   — pose stamps
"""

from __future__ import annotations

from typing import Annotated, Any

from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.tools import InjectedToolCallId, tool as lc_tool
from langgraph.types import Command

from dimos.memory2.experimental.memory2_agent.map_view import (
    MapRenderer,
    _annotate_query_in_frame,
    add_points_to_space,
    encode_space_as_multimodal,
    encode_walkthrough_blocks,
    frames_that_could_see_point,
    recall_view as _recall_view_impl,
    verify_room_partition as _verify_room_partition_impl,
    walkthrough_frames,
    walkthrough_timestamps_only,
)
from dimos.memory2.experimental.memory2_agent import skills_registry

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image as DimosImage


_KNOWN_STREAMS = {
    "color_image",
    "color_image_embedded",
    "lidar",
    "odom",
}


def _fmt_pose(pose: Any) -> str:
    if pose is None:
        return "—"
    if len(pose) >= 3:
        return f"({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
    return repr(pose)


def _fmt_data(data: Any) -> str:
    """Truncate data into a printable string for tool output."""
    if isinstance(data, str):
        return repr(data)
    if isinstance(data, DimosImage):
        return f"[Image {data.width}x{data.height} ts={data.ts:.2f}]"
    cls = type(data).__name__
    s = repr(data)
    if len(s) > 80:
        s = s[:77] + "..."
    return f"{cls}: {s}"


def _fmt_obs(obs: Observation, *, with_sim: bool = False) -> str:
    sim = ""
    if with_sim and getattr(obs, "similarity", None) is not None:
        sim = f" sim={float(obs.similarity):.3f}"
    return (
        f"id={obs.id} ts={obs.ts:.2f}{sim} pose={_fmt_pose(obs.pose)} "
        f"tags={obs.tags} data={_fmt_data(obs.data)}"
    )


def _fmt_obs_list(obs_list: list[Observation], header: str = "", *, with_sim: bool = False) -> str:
    if not obs_list:
        return f"{header}(no matches)" if header else "(no matches)"
    body = "\n".join(_fmt_obs(o, with_sim=with_sim) for o in obs_list)
    return f"{header}\n{body}" if header else body


def _validate_stream(name: str) -> str | None:
    """Return an error string if the stream name is invalid, else None."""
    if name not in _KNOWN_STREAMS:
        return f"unknown stream {name!r}; available: {sorted(_KNOWN_STREAMS)}"
    return None


def _multimodal_command(
    tool_call_id: str,
    *,
    tool_summary: str,
    image_blocks: list[dict[str, Any]],
    image_intro: str,
) -> Command:
    """Return a Command that appends a text ToolMessage + a HumanMessage
    carrying the image artifact(s).

    Bypasses the limitation that some models (gpt-5 family) don't read
    image_url blocks stuffed inside ToolMessage content. Images come
    back as a regular HumanMessage which all vision-capable chat models
    process reliably.

    The HumanMessage is tagged with `additional_kwargs["tool_call_id"]`
    so the state reducer can pair it with the right ToolMessage when
    multiple parallel tool calls return images in one batch.
    """
    msgs: list[Any] = [ToolMessage(content=tool_summary, tool_call_id=tool_call_id)]
    if image_blocks:
        msgs.append(
            HumanMessage(
                content=[{"type": "text", "text": image_intro}, *image_blocks],
                additional_kwargs={"tool_call_id": tool_call_id},
            )
        )
    return Command(update={"messages": msgs})


def build_tools(
    store: SqliteStore, clip: CLIPModel
) -> tuple[list[Any], dict[str, Any]]:
    """Build the LangChain tool list bound to a given store + CLIP model.

    Returns (tools, state) — `state` is retained for backwards-compat but
    is currently unused (no per-question state needs to be plumbed back
    to the harness). The final answer is the last assistant message.
    """
    map_renderer = MapRenderer(store)
    state: dict[str, Any] = {}

    # Per-question Monty REPL — state (variables, function defs) persists
    # across `calc` calls within one question.
    import pydantic_monty as _pm

    _calc_repl = _pm.MontyRepl()

    @lc_tool
    def calc(code: str) -> str:
        """Run Python code in a sandboxed REPL for arithmetic and
        bookkeeping. State (variables, function defs) persists across
        calls within this question.

        Use this whenever you need to compute something deterministically
        instead of doing math in your head — averages, min/max, bounding
        boxes, distances, sums over lists, etc.

        IMPORTANT — this is NOT regular Python. It is the Monty
        sandboxed interpreter. Supported:
          - operators: + - * / // % ** == != < <= > >= and or not & | ^ ~ << >>
          - builtins: min, max, sum, len, abs, round, sorted, range,
            enumerate, zip, map, filter, list, tuple, set, dict, str,
            int, float, bool, print
          - control flow: if / else / for / while / def / return /
            comprehensions / f-strings
          - small stdlib subset: re, datetime, json, typing

        NOT supported:
          - `math` module → no sin, cos, sqrt, pi (use `x ** 0.5` for
            square root)
          - third-party libraries (no numpy, sympy, pandas, etc.)
          - `import` of anything not in the supported stdlib subset
          - filesystem / network / env access

        The return value is the value of the last expression in your
        code (REPL-style). Stdout from `print(...)` is also captured
        and returned to you.
        """
        captured: list[str] = []

        def _cb(stream: str, text: str) -> None:
            if stream == "stdout":
                captured.append(text)

        try:
            result = _calc_repl.feed_run(code, print_callback=_cb)
        except Exception as e:  # noqa: BLE001
            return f"calc error: {type(e).__name__}: {e}"

        out_parts = []
        stdout_text = "".join(captured).rstrip()
        if stdout_text:
            out_parts.append(f"stdout:\n{stdout_text}")
        if result is not None:
            out_parts.append(f"result: {result!r}")
        if not out_parts:
            return "(no output, no return value)"
        return "\n".join(out_parts)

    @lc_tool
    def list_skills() -> str:
        """List the available skills (named procedures), each with a short
        description of when it applies. Returns names + descriptions only.
        """
        skills = skills_registry.list_skills()
        if not skills:
            return "No skills available."
        body = "\n".join(f"  - {s.name}: {s.description}" for s in skills)
        return "Available skills:\n" + body

    @lc_tool
    def load_skill(name: str) -> str:
        """Return the full procedure for a named skill."""
        s = skills_registry.load_skill(name)
        if s is None:
            available = ", ".join(sk.name for sk in skills_registry.list_skills()) or "(none)"
            return f"No skill named {name!r}. Available: {available}"
        return f"# skill: {s.name}\n{s.description}\n\n{s.body}"

    @lc_tool
    def list_streams() -> str:
        """List the memory streams available to query, with item counts.

        Call this first to orient yourself before issuing other queries.
        """
        names = store.list_streams()
        parts = []
        for n in names:
            try:
                s = store.stream(n)
                parts.append(f"  {n}: {s.count()} items")
            except Exception as e:  # noqa: BLE001
                parts.append(f"  {n}: <error: {e}>")
        return "Available streams:\n" + "\n".join(parts)

    @lc_tool
    def summary(stream: str) -> str:
        """Return a one-line summary of a stream: count and time range."""
        if err := _validate_stream(stream):
            return err
        return store.stream(stream).summary()

    @lc_tool
    def recent(stream: str, n: int = 5) -> str:
        """Return the n most recent observations from a stream (metadata only)."""
        if err := _validate_stream(stream):
            return err
        n = max(1, min(n, 50))
        obs = store.stream(stream).order_by("ts", desc=True).limit(n).to_list()
        return _fmt_obs_list(obs, header=f"{n} most recent in {stream!r}:")

    @lc_tool
    def search_semantic(stream: str, query: str, k: int = 5) -> str:
        """CLIP-embedding semantic search over a stream.

        The only embedded stream is `color_image_embedded`. The query
        string is embedded with CLIP text encoder and compared by cosine
        similarity. Use this for "where did I see a person", "find me a
        chair", "any windows in the recording" type questions.
        """
        if err := _validate_stream(stream):
            return err
        k = max(1, min(k, 20))
        try:
            qe = clip.embed_text(query)
            obs = store.stream(stream).search(qe, k=k).to_list()
        except Exception as e:  # noqa: BLE001
            return f"search_semantic failed on {stream!r}: {e}"
        header = (
            f"search_semantic({stream!r}, {query!r}) top {k}:\n"
            f"  (to view a hit: show_image(stream='color_image', ts=<ts>))"
        )
        return _fmt_obs_list(obs, header=header, with_sim=True)

    @lc_tool
    def near(stream: str, x: float, y: float, radius: float = 2.0, k: int = 10) -> str:
        """Spatial filter: return observations whose pose is within `radius` of (x, y).

        Z is ignored. Useful for queries like "what's near the entry door".
        """
        if err := _validate_stream(stream):
            return err
        k = max(1, min(k, 50))
        try:
            obs = (
                store.stream(stream)
                .near((float(x), float(y), 0.0), float(radius))
                .limit(k)
                .to_list()
            )
        except Exception as e:  # noqa: BLE001
            return f"near failed on {stream!r}: {e}"
        return _fmt_obs_list(
            obs, header=f"near({stream!r}, x={x}, y={y}, r={radius}) up to {k}:"
        )

    @lc_tool
    def show_image(
        stream: str,
        ts: float,
        tool_call_id: Annotated[str, InjectedToolCallId],
    ) -> Any:
        """Fetch the recorded image closest to *ts* and return it inline.

        Only works on image streams (color_image, color_image_embedded).
        The image is appended to the conversation as a follow-up
        HumanMessage so vision-capable LLMs see the actual frame, not
        just metadata.

        Use ts (not an id) because timestamps are stream-independent: take
        the ts of any hit returned by other tools and pass it here.
        """
        if err := _validate_stream(stream):
            return err
        if stream not in ("color_image", "color_image_embedded"):
            return f"show_image only supports color_image / color_image_embedded, got {stream!r}"
        try:
            all_obs = store.stream(stream).to_list()
            if not all_obs:
                return f"stream {stream!r} is empty"
            obs = min(all_obs, key=lambda o: abs(o.ts - float(ts)))
        except Exception as e:  # noqa: BLE001
            return f"show_image failed: {e}"
        summary = (
            f"frame from {stream!r}  ts={obs.ts:.2f}  pose={_fmt_pose(obs.pose)} "
            "(image follows as next user message)"
        )
        return _multimodal_command(
            tool_call_id,
            tool_summary=summary,
            image_blocks=obs.data.agent_encode(),
            image_intro=f"Image from show_image  ts={obs.ts:.2f}  pose={_fmt_pose(obs.pose)}",
        )

    @lc_tool
    def recall_view(
        at_ts: str,
        direction: str | None = None,
        yaw_deg: float | None = None,
        k: int = 3,
        tool_call_id: Annotated[str, InjectedToolCallId] = "",
    ) -> Any:
        """Return up to k recorded camera frames matching a memory query
        defined by (position, direction).

        Position comes from `at_ts`: "now" / "start" / numeric
        seconds-from-start (small) / unix ts (> 1e9). The robot pose at
        that ts gives both the query position and a body-frame heading.

        Direction — specify EXACTLY ONE:
          - direction:    "forward" / "back" / "left" / "right" — relative
                          to the body heading.
          - yaw_deg:      absolute world-frame heading in degrees.

        Frames returned in rank order with metadata (ts, time delta from
        anchor, yaw diff, xy distance). Useful for "what was on my right
        at t=30s" and similar memory questions. Top-k de-duplicated in
        time so spaced visits surface separately (good for spotting
        updates).
        """
        try:
            matches, desc = _recall_view_impl(
                store, map_renderer,
                at_ts=at_ts,
                direction=direction, yaw_deg=yaw_deg, k=k,
            )
        except ValueError as e:
            return f"recall_view: {e}"

        if not matches:
            return f"{desc}\n(no frames passed the filters)"

        image_blocks: list[dict[str, Any]] = []
        captions: list[str] = []
        for i, m in enumerate(matches, 1):
            dt_txt = f"Δt={m.time_diff_s:+.1f}s"
            caption = (
                f"frame {i}/{len(matches)}  "
                f"id={m.obs.id} ts={m.obs.ts:.2f} {dt_txt}  "
                f"yaw_diff={m.yaw_diff_deg:.1f}°  xy_dist={m.xy_dist_m:.2f}m  "
                f"score={m.score:.2f}"
            )
            captions.append(caption)
            image_blocks.append({"type": "text", "text": caption})
            image_blocks.extend(m.obs.data.agent_encode())
        summary = f"{desc}\n" + "\n".join(captions) + "\n(images follow)"
        return _multimodal_command(
            tool_call_id,
            tool_summary=summary,
            image_blocks=image_blocks,
            image_intro=desc,
        )

    @lc_tool
    def verify_room_partition(
        rooms: list[dict[str, Any]],
        points: list[dict[str, Any]] | None = None,
        tool_call_id: Annotated[str, InjectedToolCallId] = "",
    ) -> Any:
        """Render the occupancy map with your proposed room POLYGONS
        overlaid semi-transparently, marking spots the robot VISITED
        (odom) or SAW (lidar-free cells) that fall outside every
        polygon. Also detects and reports overlaps — rooms should NOT
        overlap each other.

        Arg `rooms`: a list of dicts, each with:
          - `polygon`: [[x, y], [x, y], ...]   (>=3 corners, world coords)
            OR
          - `rect`: [x_min, y_min, x_max, y_max]  (shorthand: an
            axis-aligned 4-corner polygon)
          - `id`   (optional): your label for the room (1, "A", ...)
          - `desc` (optional): short description ("conference area")

        Use POLYGONS, not rectangles, when a room is not box-shaped —
        rooms in real spaces are rarely axis-aligned to the map.

        Returned image:
          - room polygons tinted with distinct colours (~35% opacity)
            and outlined + labelled,
          - **red wash** where polygons overlap cells the lidar never
            observed (UNKNOWN cells) — your polygon is overclaiming
            into unobserved space; shrink it to follow the walls,
          - small GREEN dots = odom samples INSIDE some room,
          - small RED dots = odom samples OUTSIDE every room,
          - ORANGE markers = salient blobs of lidar-visible free
            space outside every room.

        Returned text:
          - per-room: polygon area (m^2), odom samples inside, lidar
            free-cell count inside, AND any overlap (m^2) with other
            rooms — overlaps are a problem, fix them by shrinking one
            polygon so they meet at an edge instead of cross.
          - totals: odom samples outside any room, count of salient
            outside blobs.

        Use this to verify your partition is COMPLETE (few red dots,
        few orange markers in the explored area) and DISJOINT (no
        overlaps between rooms).

        Argument `points` (optional): same schema as `show_map`'s
        `points` — list of {x, y, label?, color?} dicts. Useful for
        marking room centroids, candidate corners you're considering,
        landmark positions, etc. Each appears as a coloured numbered
        dot on the map; the legend is appended to the returned text.
        Allowed colours: red, green, blue, yellow, cyan, magenta,
        orange, white. Default: yellow.
        """
        try:
            space, stats, n_out_odom, n_total_odom, n_out_vis = _verify_room_partition_impl(
                store, map_renderer, rooms
            )
        except Exception as e:  # noqa: BLE001
            return f"verify_room_partition failed: {e}"

        if not stats:
            return "verify_room_partition: no valid rectangles provided"

        any_overlap = any(s.overlap_with for s in stats)
        any_overclaim = any(s.overclaim_m2 > 1.0 for s in stats)
        flags = []
        if any_overlap: flags.append("OVERLAPS")
        if any_overclaim: flags.append("OVERCLAIM (polygon over UNKNOWN cells)")
        lines = [
            f"verify_room_partition: {len(stats)} room(s)"
            + (f"  ⚠ {' / '.join(flags)} — see per-room list" if flags else ""),
            f"  odom samples outside any room: {n_out_odom} / {n_total_odom} sampled",
            f"  salient lidar-visible blobs outside rooms: {n_out_vis}",
            "",
            "per-room:",
        ]
        for s in stats:
            poly_repr = ", ".join(f"({x:+.2f},{y:+.2f})" for (x, y) in s.polygon)
            overclaim_pct = (s.overclaim_m2 / s.area_m2 * 100.0) if s.area_m2 > 0 else 0.0
            lines.append(
                f"  #{s.id}  area={s.area_m2:6.1f} m^2  "
                f"polygon=[{poly_repr}]"
            )
            lines.append(
                f"        odom_in={s.odom_samples_inside}  "
                f"free_cells_in={s.n_visible_inside}  "
                f"unknown_in={s.n_unknown_inside} "
                f"(overclaim {s.overclaim_m2:.1f} m^2, {overclaim_pct:.0f}%)  {s.desc}"
            )
            if s.overlap_with:
                ov = ", ".join(f"#{rid}: {area:.1f} m^2" for rid, area in s.overlap_with.items())
                lines.append(f"        ⚠ overlap with → {ov}")
            if s.overclaim_m2 > 1.0:
                lines.append(
                    f"        ⚠ overclaim: {s.overclaim_m2:.1f} m^2 of this polygon is "
                    f"UNKNOWN to lidar (shown red on map) — shrink the polygon"
                )
        points_legend = add_points_to_space(space, points, map_renderer._grid)
        if points_legend:
            lines.append("")
            lines.append(points_legend)
        header = "\n".join(lines)
        map_blocks = encode_space_as_multimodal(
            space, header, width_px=map_renderer.render_target_width()
        )
        return _multimodal_command(
            tool_call_id,
            tool_summary=header + "\n(annotated map follows)",
            image_blocks=[map_blocks[1]],
            image_intro=header,
        )

    @lc_tool
    def frames_facing(
        x: float,
        y: float,
        k: int = 4,
        max_range_m: float = 8.0,
        check_occlusion: bool = True,
        points: list[dict[str, Any]] | None = None,
        tool_call_id: Annotated[str, InjectedToolCallId] = "",
    ) -> Any:
        """Return recorded camera frames whose 2D viewing cone could
        contain the world-frame point (x, y).

        For each `color_image` frame, computes the camera's pose and
        yaw, then checks whether (x, y) falls inside the head camera's
        ~76° horizontal viewing cone within `max_range_m` metres. If
        `check_occlusion` is True (default), candidate frames whose
        line-of-sight passes through an OCCUPIED cell of the global
        occupancy grid are dropped.

        Use this when you want to inspect a place the robot may have
        looked at but never walked to (e.g. an end wall, a far corner,
        an object in the distance). The agent gets:
          1. an annotated top-down map showing the query point and the
             surviving viewing cones, and
          2. up to k of the camera frames themselves.

        Returns top-k by combined angular+distance score, time-deduped
        so distinct visits surface separately.

        Argument `points` (optional): same schema as `show_map`'s
        `points` — list of {x, y, label?, color?} dicts. Drawn on the
        top-down map as coloured numbered dots in addition to the
        primary query (yellow X) and viewing cones. Allowed colours:
        red, green, blue, yellow, cyan, magenta, orange, white.
        """
        try:
            picks, map_space = frames_that_could_see_point(
                store, map_renderer,
                x=float(x), y=float(y), k=int(k),
                max_range_m=float(max_range_m),
                check_occlusion=bool(check_occlusion),
            )
        except Exception as e:  # noqa: BLE001
            return f"frames_facing: {e}"

        if not picks:
            summary = (
                f"frames_facing(x={x:.2f}, y={y:.2f}): no recorded camera frame "
                f"had ({x:.2f}, {y:.2f}) within its viewing cone "
                f"(range<{max_range_m} m, occlusion={'on' if check_occlusion else 'off'})"
            )
            if map_space is None:
                return summary
            points_legend = add_points_to_space(map_space, points, map_renderer._grid)
            if points_legend:
                summary += "\n" + points_legend
            map_blocks = encode_space_as_multimodal(
                map_space, summary, width_px=map_renderer.render_target_width()
            )
            return _multimodal_command(
                tool_call_id,
                tool_summary=summary + " (annotated map follows)",
                image_blocks=[map_blocks[1]],
                image_intro=summary,
            )

        header = (
            f"frames_facing(x={x:.2f}, y={y:.2f}): {len(picks)} frames "
            f"could see this point (range<{max_range_m} m, "
            f"occlusion={'on' if check_occlusion else 'off'})"
        )
        per_frame_captions = []
        image_blocks: list[dict[str, Any]] = []
        # First the annotated map (with optional extra points overlaid)
        points_legend = ""
        if map_space is not None:
            points_legend = add_points_to_space(map_space, points, map_renderer._grid)
            map_blocks = encode_space_as_multimodal(
                map_space, header, width_px=map_renderer.render_target_width()
            )
            image_blocks.append(
                {"type": "text", "text": "Map: query point (yellow X) + viewing cones"}
            )
            image_blocks.append(map_blocks[1])
        # Then each candidate, with the query position projected into the frame.
        import base64 as _b64
        import cv2 as _cv2
        for i, c in enumerate(picks, 1):
            cap = (
                f"frame {i}/{len(picks)}  ts={c.obs.ts:.2f}  "
                f"cam_pose=({c.obs.pose[0]:+.2f}, {c.obs.pose[1]:+.2f})  "
                f"dist={c.distance_m:.2f} m  ang_offset={c.angular_offset_deg:.1f}°  "
                f"score={c.score:.2f}  (red cross marks the query projection)"
            )
            per_frame_captions.append(cap)
            image_blocks.append({"type": "text", "text": cap})
            annotated = _annotate_query_in_frame(c.obs.data, c.obs.pose, float(x), float(y))
            if annotated is None:
                image_blocks.extend(c.obs.data.agent_encode())
            else:
                ok, buf = _cv2.imencode(".jpg", annotated, [_cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    b64 = _b64.b64encode(bytes(buf)).decode("ascii")
                    image_blocks.append({
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                    })
                else:
                    image_blocks.extend(c.obs.data.agent_encode())
        summary_parts = [header] + per_frame_captions
        if points_legend:
            summary_parts.append(points_legend)
        summary_parts.append("(annotated map + per-frame images follow)")
        summary = "\n".join(summary_parts)
        return _multimodal_command(
            tool_call_id,
            tool_summary=summary,
            image_blocks=image_blocks,
            image_intro=header,
        )

    @lc_tool
    def walkthrough_timestamps(
        t_start: str, t_end: str, step_seconds: float = 2.0
    ) -> str:
        """Return evenly-spaced timestamps across a time range at ~step_seconds.

        Output is index → ts only, no images and no pose. Use it as a
        SCHEDULE: get the timestamps, then call show_image once per
        index to inspect each frame one at a time. The combination is
        useful when you want to reason pairwise over the walk without
        dumping every frame into context at once.

        `step_seconds` (default 2.0, min 0.5, max 3.0) controls density.
        The tool refuses any range that would require more than 32
        samples, and refuses step_seconds > 3.0 — for long recordings
        you MUST issue multiple calls over consecutive sub-ranges (e.g.
        0–60s, 60–120s, …) rather than stretching one call across the
        whole walk. At step=2.0 a single call covers up to ~62 s; at
        step=3.0 up to ~93 s.

        `t_start` / `t_end` accept the same formats as show_map's
        `when`: "now" / "start" / numeric seconds-from-start (small) /
        unix ts (> 1e9). Order doesn't matter — earlier/later auto-swap.
        """
        result = walkthrough_timestamps_only(
            store, t_start=t_start, t_end=t_end, step_seconds=step_seconds
        )
        if isinstance(result, str):
            return result
        lines = [
            f"walkthrough_timestamps: {len(result)} frames "
            f"from t_start={t_start!r} to t_end={t_end!r}"
        ]
        for idx, ts, t_rel in result:
            lines.append(f"  {idx}: ts={ts:.2f}  (t={t_rel:5.1f}s)")
        return "\n".join(lines)

    @lc_tool
    def walkthrough(
        t_start: str,
        t_end: str,
        step_seconds: float = 2.0,
        tool_call_id: Annotated[str, InjectedToolCallId] = "",
    ) -> Any:
        """Return a low-res frame sequence sampled across a time range.

        Samples evenly-spaced camera frames from `color_image` between
        `t_start` and `t_end` at ~`step_seconds` spacing, downsizes each,
        and annotates the upper-left with that frame's time, position,
        and yaw.

        `step_seconds` (default 2.0, min 0.5, max 3.0) controls density.
        The tool refuses any range that would require more than 16
        frames, and refuses step_seconds > 3.0 — for long recordings
        you MUST issue multiple calls over consecutive sub-ranges
        rather than stretching one call across the whole walk. At
        step=2.0 a single call covers up to ~30 s; at step=3.0 up to
        ~45 s.

        `t_start` and `t_end` accept the same formats as show_map's
        `when`.
        """
        result = walkthrough_frames(
            store, t_start=t_start, t_end=t_end, step_seconds=step_seconds
        )
        if isinstance(result, str):
            return result
        if not result:
            return "walkthrough: no frames sampled"
        header = (
            f"walkthrough: {len(result)} frames sampled from t_start={t_start!r} "
            f"to t_end={t_end!r}"
        )
        blocks = encode_walkthrough_blocks(result, header=header)
        # First block is the header text; rest are alternating captions + images.
        text_summary = (
            f"{header} (images and per-frame captions follow as next user message)"
        )
        return _multimodal_command(
            tool_call_id,
            tool_summary=text_summary,
            image_blocks=blocks[1:],  # drop the header text — we already used it
            image_intro=header,
        )

    @lc_tool
    def show_map(
        when: str = "now",
        points: list[dict[str, Any]] | None = None,
        tool_call_id: Annotated[str, InjectedToolCallId] = "",
    ) -> Any:
        """Render the top-down occupancy map with the agent's position and
        heading at a given moment, and return it as an inline image.

        The map shows occupied cells (walls/obstacles) in magenta, free
        space in blue, and unknown cells in black — all derived from the
        recording's lidar stream. The agent is drawn as a red arrow inside
        a white circle, pointing in its facing direction.

        Argument `when` selects the moment:
          - "now" / "end" / "latest"   -> last odom pose in the recording
          - "start" / "first"          -> first odom pose
          - "<number>"                 -> seconds-from-start (small numbers)
                                          or unix timestamp (numbers > 1e9);
                                          the closest odom pose is used.

        Argument `points` (optional): a list of dicts each with
            {x: float, y: float, label?: str, color?: str}
        Each appears on the map as a small coloured numbered dot. The
        legend (index → colour, world coord, label) is appended to the
        returned text. Allowed colours: red, green, blue, yellow,
        cyan, magenta, orange, white. Default: yellow. Use this to
        mark landmarks, candidate positions, computed centroids, etc.
        Soft cap: 16 points.
        """
        result = map_renderer.space(when)
        if result is None:
            return f"show_map: could not resolve when={when!r}"
        space, ts, pose = result
        points_legend = add_points_to_space(space, points, map_renderer._grid)
        caption = (
            f"top-down map at when={when!r} (ts={ts:.2f}, "
            f"pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}))"
        )
        if points_legend:
            caption_with_legend = caption + "\n" + points_legend
        else:
            caption_with_legend = caption
        blocks = encode_space_as_multimodal(
            space, caption_with_legend, width_px=map_renderer.render_target_width()
        )
        return _multimodal_command(
            tool_call_id,
            tool_summary=caption_with_legend + "\n(image follows)",
            image_blocks=[blocks[1]],
            image_intro=caption,
        )

    return [
        calc,
        list_skills,
        load_skill,
        list_streams,
        summary,
        recent,
        search_semantic,
        near,
        show_image,
        show_map,
        walkthrough,
        walkthrough_timestamps,
        recall_view,
        frames_facing,
        verify_room_partition,
    ], state
