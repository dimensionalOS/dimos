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

"""Text in, ranked frames out: the step both answers start from.

Whether the answer is built by grouping patch pyramids in 3D or by handing one good
image to a detector, the first move is the same -- ask each model's vector stream which
patches match the words, and collect them by the frame they came from. Nothing is
loaded up front: a patch row in the flat layout already carries its frame, its stamp,
its ray and its depth, so ranking frames costs one search per model plus a read of the
winning rows' vectors, not a pass over the recording.

Frames are then split into *episodes*. The trolley passes a shelf, sees the thing for a
few seconds and moves on; a second pass down the same aisle is a second episode of the
same object, which is two chances at it rather than one thing counted twice.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace.ingest import patch_stream_for
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

# Subtracted from every score. Without them a query scores highest on whatever the
# camera sees most of, which in a supermarket is shelving and floor. The list is fixed,
# so its vectors are the same for every query a process ever runs -- hence the cache
# below, which is most of the sixteen seconds a four-model query used to spend encoding.
BACKGROUND_PROMPTS = ("a floor", "a wall", "a ceiling", "a shelf", "a photo of a room")

# Rows to ask the vector index for, per model. The contrast threshold then does the
# real filtering; this only bounds the read.
DEFAULT_TOP_K = 4000

# One patch has to beat its background by this much to count. Deliberately low: the
# frames step is a *recall* step whose job is to nominate images for the detector, and
# the detector is what refuses.
DEFAULT_THRESHOLD = 0.005


@dataclass
class Hit:
    """One patch that matched, with everything needed to place it."""

    member: str
    frame: str
    ts: float
    cell: int
    grid: tuple[int, int]
    ray: tuple[float, float]
    depth: float
    score: float


@dataclass
class Frame:
    """One moment, and every model's patches that matched there."""

    frame: str
    ts: float
    hits: list[Hit] = field(default_factory=list)

    @property
    def best(self) -> float:
        """The single strongest patch. Robust to how much of the view the thing fills."""
        return max(hit.score for hit in self.hits)

    @property
    def weight(self) -> float:
        """Total match. Favours a frame where the thing is large and clearly seen."""
        return float(sum(hit.score for hit in self.hits))

    @property
    def members(self) -> set[str]:
        """Which models agree there is something here."""
        return {hit.member for hit in self.hits}


@dataclass
class Episode:
    """A run of frames with no long quiet in the middle of it: one look at one thing."""

    frames: list[Frame]
    # When the agreement step chose this episode, it also said which frame it agreed
    # hardest in. That beats total match weight at picking the frame to detect in: the
    # weight counts every hot cell, including the floor ones the agreement threw out.
    heat: dict[float, float] | None = None

    @property
    def start(self) -> float:
        return self.frames[0].ts

    @property
    def end(self) -> float:
        return self.frames[-1].ts

    @property
    def span(self) -> float:
        return self.end - self.start

    @property
    def peak(self) -> Frame:
        """The frame to hand the detector: the one with the most match in it.

        Total weight rather than the single best patch, because a frame where the thing
        is large and centred is a better image to detect in than one where a corner of
        it clips the edge with a slightly higher peak.
        """
        if self.heat:
            return max(self.frames, key=lambda frame: self.heat.get(frame.ts, 0.0))
        return max(self.frames, key=lambda frame: frame.weight)

    @property
    def score(self) -> float:
        return self.peak.weight

    @property
    def members(self) -> set[str]:
        """Every model that voted for any frame in this episode."""
        return {member for frame in self.frames for member in frame.members}

    def by_weight(self) -> list[Frame]:
        """Frames best-first, for a caller that wants to try more than one."""
        if self.heat:
            return sorted(self.frames, key=lambda frame: -self.heat.get(frame.ts, 0.0))
        return sorted(self.frames, key=lambda frame: -frame.weight)


def member_streams(store: Any) -> list[tuple[str, str]]:
    """``(member tag, stream name)`` per model, in name order.

    Skips the bare-prefix stream an older ingest left behind: an empty embeddings
    stream is a name, not an index, and adopting one by name gives a search that can
    never be built.
    """
    prefix = patch_stream_for("", "")
    found = [
        (name[len(prefix) :], name)
        for name in store.list_streams()
        if name.startswith(prefix) and name != prefix
    ]
    return sorted(found)


def spec_of(tag: str) -> str:
    """A stream's model tag back to the checkpoint that wrote it.

    ``so400m_patch16_naflex_1024`` -> ``google/siglip2-so400m-patch16-naflex@1024``.
    The stream name carries the answer because a stream of vectors is unusable without
    it -- you cannot encode the query text to compare against.
    """
    if tag.startswith("pe_"):
        # Written by a Perception Encoder checkpoint; its name is the rest of the tag,
        # with the pooling back on the end where it started.
        body = tag[3:]
        for how in ("direct",):
            if body.endswith(f"_{how}"):
                return f"pe:{body[: -len(how) - 1].replace('_', '-')}@{how}"
        return f"pe:{body.replace('_', '-')}"
    budget = ""
    head, _, tail = tag.rpartition("_")
    if tail.isdigit() and head.endswith("naflex"):
        tag, budget = head, f"@{tail}"
    return f"google/siglip2-{tag.replace('_', '-')}{budget}"


class TextTowers:
    """The text side of one or more checkpoints, loaded once and kept.

    Two caches, and the second is the point. Loading a tower costs about a second;
    encoding the five fixed background prompts cost most of a four-model query's sixteen
    seconds of text encoding, and they are identical for every query the process runs.
    """

    def __init__(self, device: str = "cpu") -> None:
        self.device = device
        self._towers: dict[str, Any] = {}
        self._background: dict[str, NDArray[np.float32]] = {}
        self._queries: dict[tuple[str, str], NDArray[np.float32]] = {}

    def place(self, device: str) -> None:
        """Name the device before the first tower is built.

        Loading is lazy, which is what makes this possible: `warm` can put the detector
        and the index where they go and only then ask what is left, instead of choosing
        a device for the towers before anything else has taken its share.
        """
        if self._towers:
            raise RuntimeError("the towers are already loaded; place them before using them")
        self.device = device

    def _tower(self, spec: str) -> Any:
        from dimos.mapping.hyperspace.siglip_embedder import PatchEnsemble

        tower = self._towers.get(spec)
        if tower is None:
            tower = PatchEnsemble([spec], device=self.device, towers="text")
            tower.start()
            self._towers[spec] = tower
        return tower

    def query(self, spec: str, text: str) -> NDArray[np.float32]:
        key = (spec, text)
        if key not in self._queries:
            self._queries[key] = self._tower(spec).embed_text(text)[0]
        return self._queries[key]

    def background(self, spec: str) -> NDArray[np.float32]:
        if spec not in self._background:
            vectors = self._tower(spec).embed_text_array(*BACKGROUND_PROMPTS)[0]
            self._background[spec] = np.stack(vectors)
        return self._background[spec]

    def close(self) -> None:
        for tower in self._towers.values():
            tower.stop()
        self._towers.clear()


def hot_frames(
    store: Any,
    text: str,
    *,
    towers: TextTowers | None = None,
    models: Sequence[str] | None = None,
    top_k: int = DEFAULT_TOP_K,
    threshold: float = DEFAULT_THRESHOLD,
    device: str = "cpu",
    resident: Any = None,
    contrast: bool = True,
    background_prompts: Sequence[str] | None = None,
    rank_with: str = "",
    rank_frames: int = 0,
) -> list[Frame]:
    """Frames that matched *text*, in time order.

    The search is a matrix multiply over every patch of every model named, held in
    memory by *resident* -- exact, with no approximate index between the words and the
    answer. Going through sqlite instead was measured at fifty times slower and is
    gone; the index is loaded once, at startup or as a recording is ingested.

    *models* names the member tags to search; the default is every model in the store.

    *background_prompts* replaces what the contrast subtracts, for this call only.

    *contrast* subtracts the best of a handful of generic prompts -- floor, wall,
    ceiling, shelf, a room -- from the score, which is what stops a wall from answering
    every question moderately well. On by default for that reason. Turning it off gives
    the plain similarity to the query, which is the honest comparison to have when
    asking whether the contrast is helping on some particular recording, and the right
    answer when the thing being looked for IS a wall or a floor.
    """
    from dimos.mapping.hyperspace.resident import RESIDENT

    held_by = resident or RESIDENT
    owned = towers is None
    towers = towers or TextTowers(device)
    frames: dict[tuple[str, float], Frame] = {}
    members = [
        (tag, name) for tag, name in member_streams(store) if models is None or tag in models
    ]
    # With `rank_with`, one member is scored over everything and decides which frames are
    # worth looking at; the others are then scored ONLY over the rows belonging to those
    # frames. The whole cost of a search is reading the vectors, so this is most of a
    # multi-model search's time for a set of frames the cheap member already liked.
    # It is a real trade and not a free one: a frame only the expensive members would
    # have found is now never seen, because nothing looks there.
    if rank_with == "auto" and len(members) > 1:
        # The cheapest member to search is the one with the fewest numbers in it, which
        # is rows x width and not the smallest-sounding name -- bike.db's two naflex
        # members have the same row count and differ 768 against 1152 in width.
        rank_with = min(
            (tag for tag, _ in members),
            key=lambda tag: _search_cost(held_by, store, tag, dict(members)[tag]),
        )
    if rank_with and len(members) > 1 and any(tag == rank_with for tag, _ in members):
        members.sort(key=lambda member: member[0] != rank_with)
    else:
        rank_with = ""
    allowed: set[tuple[str, float]] | None = None
    try:
        for tag, name in members:
            spec = spec_of(tag)
            query = towers.query(spec, text)
            if not contrast:
                background = np.empty((0, len(query)), np.float32)
            elif background_prompts is None:
                background = towers.background(spec)
            else:
                # An area query is contrasted against objects rather than against the
                # room, so that asking for a room does not subtract the room.
                background = np.stack([towers.query(spec, prompt) for prompt in background_prompts])

            held = held_by.of(store, tag, name)
            rows = None if allowed is None else _rows_on(held, allowed)
            if rows is not None and not len(rows):
                continue
            picked, scored = held.hot(
                query, background, threshold=threshold, limit=top_k, rows=rows
            )
            for index, score in zip(picked, scored, strict=True):
                key = (held.camera_frames[held.frame_of[index]], float(held.ts[index]))
                frame = frames.get(key)
                if frame is None:
                    frame = frames[key] = Frame(frame=key[0], ts=key[1])
                frame.hits.append(
                    Hit(
                        member=tag,
                        frame=key[0],
                        ts=key[1],
                        cell=int(held.cell[index]),
                        grid=(int(held.grid[index][0]), int(held.grid[index][1])),
                        ray=(float(held.ray[index][0]), float(held.ray[index][1])),
                        depth=float(held.depth[index]),
                        score=float(score),
                    )
                )
            if rank_with and tag == rank_with:
                # The ranking member's BEST frames, not all of them. MEASURED on bike.db:
                # its 4000 hot patches for "a stop sign" land on 1335 distinct frames of
                # 2308, so narrowing by "every frame it touched" narrows to 58% and saves
                # nothing -- search went 0.81 s -> 0.83 s, which is noise. The cut is what
                # makes this worth having; the frames are ranked by total match, the same
                # order the episode step would have put them in.
                keep = sorted(frames.values(), key=lambda frame: -frame.weight)
                if rank_frames > 0:
                    keep = keep[:rank_frames]
                allowed = {(frame.frame, frame.ts) for frame in keep}
                if not allowed:
                    # Nothing to confirm. Leaving `allowed` empty would have the other
                    # members score nothing and the query answer nothing, which is the
                    # right answer here but for the wrong reason -- say it once, loudly,
                    # rather than have a silent empty set look like a narrow one.
                    logger.info(
                        f"hyperspace: {rank_with!r} found nothing for {text!r}, "
                        "so there is nowhere for the other models to look"
                    )
                    break
    finally:
        if owned:
            towers.close()
    return sorted(frames.values(), key=lambda frame: frame.ts)


def _search_cost(held_by: Any, store: Any, tag: str, stream: str) -> int:
    """Numbers a full search of this member reads. Rows times width, not rows alone."""
    held = held_by.of(store, tag, stream)
    return int(held.rows) * int(held.width)


def _rows_on(held: Any, allowed: set[tuple[str, float]]) -> NDArray[np.intp]:
    """Row numbers of the patches belonging to *allowed* frames.

    Touches the `ts` and `frame_of` columns only -- tens of megabytes -- and never the
    vectors, which is what makes narrowing cheaper than the search it replaces.
    """
    wanted: dict[int, list[float]] = {}
    for name, ts in allowed:
        try:
            at = held.camera_frames.index(name)
        except ValueError:
            continue
        wanted.setdefault(at, []).append(ts)
    if not wanted:
        return np.empty(0, dtype=np.intp)
    mask = np.zeros(held.rows, dtype=bool)
    stamps = held.ts[: held.rows]
    frame_of = held.frame_of[: held.rows]
    for at, times in wanted.items():
        mask |= (frame_of == at) & np.isin(stamps, np.asarray(times, dtype=stamps.dtype))
    return np.flatnonzero(mask).astype(np.intp)


def episodes(frames: Sequence[Frame], gap_s: float = 1.0) -> list[Episode]:
    """Split frames into runs separated by more than *gap_s* of quiet."""
    runs: list[Episode] = []
    for frame in frames:
        if runs and frame.ts - runs[-1].frames[-1].ts <= gap_s:
            runs[-1].frames.append(frame)
        else:
            runs.append(Episode(frames=[frame]))
    return runs


def ranked_episodes(
    frames: Sequence[Frame],
    *,
    gap_s: float = 1.0,
    min_frames: int = 1,
    limit: int | None = None,
) -> list[Episode]:
    """Episodes worth handing to a detector, strongest first.

    *min_frames* drops a single stray frame: an object the camera really passed is seen
    in several consecutive frames at 4 Hz, and one lone frame is more often a model
    reaching for something that is not there.
    """
    found = [episode for episode in episodes(frames, gap_s) if len(episode.frames) >= min_frames]
    found.sort(key=lambda episode: -episode.score)
    return found if limit is None else found[:limit]


def hot_cells(frame: Frame, member: str | None = None) -> Iterator[tuple[int, int, float]]:
    """``(row, col, score)`` of this frame's matching cells, on one member's grid.

    Not used by the detector path -- the detector gets the whole image -- but it is what
    draws the heat overlay that shows *why* a frame was nominated.
    """
    for hit in frame.hits:
        if member is not None and hit.member != member:
            continue
        cols = hit.grid[1]
        yield hit.cell // cols, hit.cell % cols, hit.score
