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

# SKETCH — not functional. Full module examples in the generic structure from
# puremodule2.py: PureModule[TIn, TOut], top-level In/Out dataclasses whose fields
# carry both type and sampler/contract, and a single `fold(self, inputs)` where
# each `step` is a typed Step[TIn, TOut] (step.inputs.* reads, step.out.* writes).

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos.memory2.puremodule2 import (
    PureModule,
    Step,
    contract,
    interpolate,
    latest,
    tick,
)

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


# ══════════════════════════════════════════════════════════════════════════════
# 1. Tagger — multi-in → single-out. pose REQUIRED (non-optional), so no guard;
#    the engine skips steps until pose exists. image fires; pose lerps to its ts.
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class TaggerIn:
    image: Image = tick(expect_hz=30)
    pose: PoseStamped = interpolate()


@dataclass
class TaggerOut:
    located: str = contract(min_hz=10)


class Tagger(PureModule[TaggerIn, TaggerOut]):
    def fold(self, inputs: Iterator[Step[TaggerIn, TaggerOut]]) -> Iterator[Observation[TaggerOut]]:
        for step in inputs:  # pose guaranteed present
            i = step.inputs
            step.out.located = f"bright={i.image.brightness:.2f} @ ({i.pose.position.x:.2f}, {i.pose.position.y:.2f})"
            yield step.emit()


# ══════════════════════════════════════════════════════════════════════════════
# 2. QualityGate — single-in → multi-out, sparse. quality every step; alert only
#    on dark frames (leave it unset → that port stays silent this step).
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class QualityIn:
    image: Image = tick(expect_hz=30)


@dataclass
class QualityOut:
    quality: float = contract(min_hz=10)
    alert: str | None = None  # sparse — no contract


class QualityGate(PureModule[QualityIn, QualityOut]):
    def fold(
        self, inputs: Iterator[Step[QualityIn, QualityOut]]
    ) -> Iterator[Observation[QualityOut]]:
        for step in inputs:
            b = step.inputs.image.brightness
            step.out.quality = round(b, 3)  # every step
            if b < 0.42:
                step.out.alert = f"dark frame ({b:.2f})"  # only when dark
            yield step.emit()


# ══════════════════════════════════════════════════════════════════════════════
# 3. VoxelGridMapper — fold-with-resource. The grid is allocated in the prologue
#    (warmup), lives across every step, and is disposed in finally (teardown).
#    Single-in/out, so it also slots into a pipeline: stream.transform(VoxelGridMapper()).
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class VoxelIn:
    lidar: PointCloud2 = tick()


@dataclass
class VoxelOut:
    global_map: PointCloud2 = contract(min_hz=1)


class VoxelGridMapper(PureModule[VoxelIn, VoxelOut]):
    voxel_size: float = 0.05
    emit_every: int = 5

    def fold(self, inputs: Iterator[Step[VoxelIn, VoxelOut]]) -> Iterator[Observation[VoxelOut]]:
        grid: VoxelGrid = VoxelGrid(voxel_size=self.voxel_size, carve_columns=True)  # warmup
        try:
            for n, step in enumerate(inputs, 1):
                grid.add_frame(step.inputs.lidar)
                if n % self.emit_every == 0:
                    step.out.global_map = grid.get_global_pointcloud2()
                    yield step.emit()
        finally:
            grid.dispose()  # teardown — runs in over() and live


# ══════════════════════════════════════════════════════════════════════════════
# 4. CostMapper — multi-in → single-out. global_map fires; relocalized_map is an
#    OPTIONAL latest() (may be None), so it's guarded. Pure per-step compute, no
#    resource: the simplest shape.
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class CostIn:
    global_map: PointCloud2 = tick()
    relocalized_map: PointCloud2 | None = latest()  # optional → may be None


@dataclass
class CostOut:
    global_costmap: OccupancyGrid = contract(min_hz=2)


class CostMapper(PureModule[CostIn, CostOut]):
    algo: str = "height_cost"

    def fold(self, inputs: Iterator[Step[CostIn, CostOut]]) -> Iterator[Observation[CostOut]]:
        for step in inputs:
            msg = step.inputs.relocalized_map or step.inputs.global_map  # prefer relocalized
            step.out.global_costmap = self._costmap(msg)
            yield step.emit()

    def _costmap(self, msg: PointCloud2) -> OccupancyGrid: ...


# ══════════════════════════════════════════════════════════════════════════════
# 5. RelocalizationModule — stateful + multi-out + side effect. The world→map
#    fix and last-reloc time are just LOCALS (no Mealy state). premap loads in the
#    prologue (warmup). Relocalize is throttled via step.ts; the TF broadcast is a
#    deliberate effect; loaded_map is emitted only when configured (sparse).
# ══════════════════════════════════════════════════════════════════════════════
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50_000


@dataclass
class RelocIn:
    global_map: PointCloud2 = tick()


@dataclass
class RelocOut:
    relocalized_map: PointCloud2 = contract(min_hz=1)
    loaded_map: PointCloud2 | None = None  # sparse — viz only


class RelocalizationModule(PureModule[RelocIn, RelocOut]):
    map_file: str | None = None
    publish_loaded_map: bool = False

    def fold(self, inputs: Iterator[Step[RelocIn, RelocOut]]) -> Iterator[Observation[RelocOut]]:
        premap = self._load_premap()  # warmup — read the prebuilt map
        if premap is None:
            return  # disabled: no map_file
        world_to_map: Transform | None = None  # "state" — just locals
        last_reloc_ts = float("-inf")
        for step in inputs:
            scan = step.inputs.global_map
            if step.ts - last_reloc_ts >= RELOC_INTERVAL and len(scan) >= MIN_LOCAL_POINTS:
                last_reloc_ts = step.ts
                world_to_map = self._try_relocalize(scan) or world_to_map
            if world_to_map is None:
                continue  # silent until first fix
            self.tf.publish(world_to_map.now())  # deliberate side effect
            step.out.relocalized_map = self._merge(scan, world_to_map, premap)
            if self.publish_loaded_map:
                step.out.loaded_map = premap  # else stays silent
            yield step.emit()

    def _load_premap(self) -> PointCloud2 | None: ...
    def _try_relocalize(self, scan: PointCloud2) -> Transform | None: ...
    def _merge(self, scan: PointCloud2, tf: Transform, premap: PointCloud2) -> PointCloud2: ...


# ══════════════════════════════════════════════════════════════════════════════
# 6. Captioner — ASYNC fold. Write `async def fold` (an async generator) when the
#    per-step work is I/O-bound and you want CONCURRENCY: fire VLM calls without
#    waiting, emit each caption as it lands. The engine detects the async form and
#    drives it on a per-module event loop. Warmup/finally/parking are unchanged.
#    (Rule: async fold = I/O concurrency; never put blocking compute in it.)
# ══════════════════════════════════════════════════════════════════════════════
class VLMClient:  # placeholder async I/O client
    async def caption(self, image: Image) -> str: ...
    async def aclose(self) -> None: ...


@dataclass
class CaptionIn:
    image: Image = tick(expect_hz=2)


@dataclass
class CaptionOut:
    caption: str = contract(min_hz=1)


class Captioner(PureModule[CaptionIn, CaptionOut]):
    max_inflight: int = 4

    async def fold(
        self, inputs: AsyncIterator[Step[CaptionIn, CaptionOut]]
    ) -> AsyncIterator[Observation[CaptionOut]]:
        client = VLMClient()  # warmup
        pending: set[asyncio.Task[Observation[CaptionOut]]] = set()
        try:
            async for step in inputs:
                pending.add(asyncio.create_task(self._caption(client, step)))  # fire, don't block
                if len(pending) >= self.max_inflight:
                    done, pending = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
                    for t in done:
                        yield t.result()  # emit as they finish
            for r in await asyncio.gather(*pending):  # drain the tail
                yield r
        finally:
            await client.aclose()  # teardown

    async def _caption(
        self, client: VLMClient, step: Step[CaptionIn, CaptionOut]
    ) -> Observation[CaptionOut]:
        step.out.caption = await client.caption(step.inputs.image)
        return step.emit()


# ══════════════════════════════════════════════════════════════════════════════
# CALLERS — the three deployment modes, unchanged across every module above.
# ══════════════════════════════════════════════════════════════════════════════
def _offline_over() -> None:
    # multi-input: name a stream per input → lazy output Stream
    for obs in CostMapper.over(global_map=..., relocalized_map=...):
        print(obs.data.global_costmap)


def _live() -> None:
    m = VoxelGridMapper()
    m.warmup()  # alloc grid, park on empty buffer
    m.start()  # open lidar input → frames flow
    m.stop()  # close → fold finally → grid.dispose()


# ══════════════════════════════════════════════════════════════════════════════
# AS TRANSFORMS — any SINGLE-input module IS a memory2 Transformer (its __call__
# is iterator→iterator), so it drops straight into stream.transform(...). The
# fold's prologue/finally run as the pipeline starts/ends. Multi-input modules
# can't (a transform has one upstream) — use .over(a=…, b=…) instead.
# ══════════════════════════════════════════════════════════════════════════════
def _xf_basic(store) -> None:  # type: ignore[no-untyped-def]
    # the whole point: a configured module is the transform
    global_maps = store.streams.lidar.transform(VoxelGridMapper(voxel_size=0.05))
    for obs in global_maps:  # obs.data: VoxelOut
        print(len(obs.data.global_map))


def _xf_equivalence(store) -> None:  # type: ignore[no-untyped-def]
    # for a single-input module these two are the same thing:
    a = store.streams.lidar.transform(VoxelGridMapper())
    b = VoxelGridMapper.over(lidar=store.streams.lidar)
    assert a.to_list() == b.to_list()


def _xf_chained(store) -> None:  # type: ignore[no-untyped-def]
    # compose with the stdlib transforms (downsample/throttle/save) and module
    # transforms in one pull-based pipeline — nothing runs until a terminal
    from dimos.memory2.transform import downsample

    pipeline = (
        store.streams.lidar.transform(downsample(2))  # every other scan
        .transform(VoxelGridMapper(voxel_size=0.1))  # accumulate → global_map
        .map_data(lambda m: m.global_map)  # unwrap VoxelOut → PointCloud2
    )
    pipeline.save(store.streams.global_map).drain()  # persist results back to the store


def _xf_stack(store) -> None:  # type: ignore[no-untyped-def]
    # stack single-input modules back to back — each one's output stream is the
    # next one's upstream (here a hypothetical single-input postprocess module)
    out = (
        store.streams.lidar.transform(VoxelGridMapper())  # lidar → global_map
        .map_data(lambda m: m.global_map)
        .transform(MapDenoiser())  # PointCloud2 → PointCloud2
    )
    out.to_list()


def _xf_live(store) -> None:  # type: ignore[no-untyped-def]
    # a transform pipeline can run live too — .live() before .transform()
    live_maps = store.streams.lidar.live().transform(VoxelGridMapper())
    live_maps.observable().subscribe(on_next=lambda obs: ...)  # tail forever


def _xf_multi_input_is_an_error(store) -> None:  # type: ignore[no-untyped-def]
    # CostMapper has two inputs → no single upstream → __call__ raises.
    # store.streams.global_map.transform(CostMapper())   # TypeError
    # use over() and name each stream instead:
    CostMapper.over(
        global_map=store.streams.global_map,
        relocalized_map=store.streams.relocalized_map,
    ).to_list()


class MapDenoiser(PureModule):  # tiny single-in/out helper for _xf_stack
    ...


# ══════════════════════════════════════════════════════════════════════════════
# MULTI-I/O DEPLOYMENT — the SAME multi-input module (CostMapper: global_map +
# relocalized_map → global_costmap), run two ways. The fold never changes.
# ══════════════════════════════════════════════════════════════════════════════
def _mem2_load() -> None:
    # (A) MEM2 LOAD — pull stored streams off a recording and run over() them.
    # Pull-based and deterministic: no threads, no ports, no transports. The
    # alignment (relocalized_map sampled latest at each global_map tick) replays
    # exactly as it happened.
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    store = SqliteStore(path=get_data("go2_run.db"))
    costmaps = CostMapper(algo="height_cost").over(
        global_map=store.streams.global_map,
        relocalized_map=store.streams.relocalized_map,
    )
    grids = [obs.data.global_costmap for obs in costmaps]  # obs.data: CostOut
    _ = grids


def _live_with_transports() -> None:
    # (B) LIVE + TRANSPORTS — each In/Out field becomes a pub/sub port; attach a
    # transport per port and the module talks LCM on the wire. Inputs arrive on
    # their topics, get aligned per tick, the output publishes on its topic.
    from dimos.core.transport import pLCMTransport

    m = CostMapper(algo="height_cost")
    m.global_map.transport = pLCMTransport("/map/global")  # In  (tick)
    m.relocalized_map.transport = pLCMTransport("/map/relocalized")  # In  (latest)
    m.global_costmap.transport = pLCMTransport("/costmap/global")  # Out

    m.global_costmap.subscribe(lambda grid: ...)  # any in-process consumer, too
    m.warmup()  # prologue runs, parks on empty buffer
    m.start()  # ports open → data flows → fold steps
    # ...
    m.stop()


def _multi_output_transports() -> None:
    # multi-OUTPUT (RelocalizationModule: global_map → relocalized_map + loaded_map):
    # each output is its own port with its own transport and subscribers.
    from dimos.core.transport import pLCMTransport

    m = RelocalizationModule(map_file="office_map", publish_loaded_map=True)
    m.global_map.transport = pLCMTransport("/map/global")
    m.relocalized_map.transport = pLCMTransport("/map/relocalized")
    m.loaded_map.transport = pLCMTransport("/map/loaded")
    m.warmup()
    m.start()
    m.stop()


def _live_from_recording() -> None:
    # (C) the live↔stored switch: drive the LIVE module from a recording instead
    # of the wire — set input_sources before start(). Same module, no transports.
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path="go2_run.db")
    m = CostMapper()

    m.input_sources = {
        "global_map": store.replay(speed=2.0).streams.global_map,  # wall-clock paced 2x
        "relocalized_map": store.streams.relocalized_map,
    }

    m.global_costmap.subscribe(lambda grid: ...)
    m.warmup()
    m.start()
