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

"""The combined fiducial->judge stack: composition, no-op defaults, and the wired path.

The last test is the pre-IRL gate in miniature: both modules in-process on a
shared ``world_map_fix`` LCM transport (exactly what the coordinator's
``_connect_streams`` builds for a matching Out/In name+type pair), a synthetic
tag frame in, and the assertion that RelocalizationModule's FiducialPrior ends
up holding the INVERTED fix (module boundary rule: the stream carries
world->map, the prior stores map_T_world).
"""

from __future__ import annotations

import asyncio
from pathlib import Path
import threading
import time
from uuid import uuid4

import numpy as np
import yaml

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.transport import LCMTransport
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.fiducial.test_helpers import camera_info, synthetic_marker_image
from dimos.perception.fiducial.visual_relocalization_module import VisualRelocalizationModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import (
    unitree_go2_fiducial_relocalization,
)
from dimos.utils.threadpool import get_max_workers, get_scheduler

_MARKER_ID = 7  # synthetic_marker_image's rendered tag


def _prewarm_global_scheduler() -> None:
    """Spawn every worker of the process-global ThreadPoolScheduler at import.

    ``In.observable()`` delivers through that scheduler, whose executor threads
    are deliberately process-persistent (dimos.utils.threadpool) — so the first
    test to push a message through an In stream would otherwise get blamed for
    them by conftest's per-test thread-leak check. Importing at collection time
    puts them in every test's pre-existing set instead.
    """
    n = get_max_workers()
    barrier = threading.Barrier(n + 1)
    for _ in range(n):
        get_scheduler().executor.submit(barrier.wait)
    barrier.wait(timeout=10.0)


_prewarm_global_scheduler()


def test_unitree_go2_fiducial_relocalization_composes_full_path() -> None:
    bp = unitree_go2_fiducial_relocalization
    assert isinstance(bp, Blueprint)

    modules = [b.module for b in bp.blueprints]
    assert RelocalizationModule in modules
    assert VisualRelocalizationModule in modules

    reloc = next(b for b in bp.blueprints if b.module is RelocalizationModule)
    vis = next(b for b in bp.blueprints if b.module is VisualRelocalizationModule)
    assert reloc.kwargs == {"use_fiducial_prior": True}  # map_file stays unset -> no-op
    assert vis.kwargs["marker_length_m"] == 0.10
    assert vis.kwargs["camera_info"].frame_id == "camera_optical"
    assert "marker_map_file" not in vis.kwargs  # stays unset -> no-op

    # Reproduce ModuleCoordinator._connect_streams' grouping key: one shared
    # (name, type) entry means the visual Out and the reloc In land on the
    # same transport — the autoconnect wiring the whole blueprint exists for.
    members: dict[tuple[str, type], set[tuple[str, str]]] = {}
    for atom in bp.active_blueprints:
        for stream in atom.streams:
            name = bp.remapping_map.get((atom.name, stream.name), stream.name)
            if isinstance(name, str):
                members.setdefault((name, stream.type), set()).add((atom.name, stream.direction))
    assert members[("world_map_fix", Transform)] == {
        ("visualrelocalizationmodule", "out"),
        ("relocalizationmodule", "in"),
    }


def _lcm(topic_ns: str, name: str, type_: type) -> LCMTransport:  # type: ignore[type-arg]
    return LCMTransport(f"/{topic_ns}/{name}", type_)


# These two are deliberately sync: a module constructed inside pytest-asyncio's
# running loop would capture it (module.get_loop), and start()'s handler
# bootstrap then deadlocks scheduling onto the loop the test itself blocks.
# Sync construction gives each module its own loop thread — the deployed shape.
def test_default_config_is_noop_until_maps_configured() -> None:
    """Blueprint defaults (no map_file, no marker_map_file) must start and idle."""
    ns = f"test_fidreloc_{uuid4().hex[:8]}"

    reloc = RelocalizationModule(use_fiducial_prior=True)
    reloc.set_transport("global_map", _lcm(ns, "global_map", PointCloud2))
    reloc.set_transport("world_map_fix", _lcm(ns, "world_map_fix", Transform))
    vis = VisualRelocalizationModule(marker_length_m=0.10, camera_info=camera_info())
    vis.set_transport("color_image", _lcm(ns, "color_image", Image))
    vis.set_transport("world_map_fix", _lcm(ns, "world_map_fix", Transform))
    try:
        reloc.start()
        vis.start()
        assert reloc._premap is None  # disabled before any map decode
        assert vis._marker_map == {}  # start() returned before load_marker_map
        # Even a frame WITH a tag in view stays a no-op on an empty map.
        asyncio.run(vis.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0)))
        assert vis.tf.get("world", "map") is None
    finally:
        vis.stop()
        reloc.stop()


def test_world_map_fix_out_reaches_fiducial_prior_inverted(tmp_path: Path) -> None:
    """Synthetic tag frame -> visual fix -> shared LCM stream -> FiducialPrior, inverted."""
    ns = f"test_fidreloc_{uuid4().hex[:8]}"
    premap_file = tmp_path / "premap.pc2.lcm"
    rng = np.random.default_rng(7)  # deterministic; printed nowhere but pinned for reruns
    premap_file.write_bytes(
        PointCloud2.from_numpy(rng.uniform(-1, 1, (64, 3)), timestamp=10.0).lcm_encode()
    )
    marker_map_file = tmp_path / "markers.yaml"
    marker_map_file.write_text(
        yaml.safe_dump(
            {"markers": {_MARKER_ID: {"translation": [0.0, 0.0, 0.0], "rotation": [0, 0, 0, 1]}}}
        )
    )

    # One transport instance per (name, type) key shared by both modules —
    # what ModuleCoordinator._connect_streams does for this blueprint.
    fix_transport = _lcm(ns, "world_map_fix", Transform)
    reloc = RelocalizationModule(use_fiducial_prior=True, map_file=str(premap_file))
    reloc.set_transport("global_map", _lcm(ns, "global_map", PointCloud2))
    reloc.set_transport("world_map_fix", fix_transport)
    vis = VisualRelocalizationModule(
        marker_length_m=0.15,  # synthetic_marker_image's rendered edge length
        camera_info=camera_info(),
        marker_map_file=str(marker_map_file),
    )
    vis.set_transport("color_image", _lcm(ns, "color_image", Image))
    vis.set_transport("world_map_fix", fix_transport)

    published: list[Transform] = []
    unsubscribe = fix_transport.subscribe(published.append)
    try:
        reloc.start()
        vis.start()
        assert reloc._premap is not None  # map_file override flips it out of no-op
        assert set(vis._marker_map) == {_MARKER_ID}

        vis.tf.publish(Transform(frame_id="world", child_frame_id="camera_optical", ts=10.0))
        asyncio.run(vis.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0)))

        deadline = time.monotonic() + 5.0
        while reloc._fiducial_prior._fix_T is None:
            assert time.monotonic() < deadline, "fix never reached the prior"
            time.sleep(0.05)

        assert len(published) == 1  # the gate passed exactly one fix onto the stream
        fix = published[0]
        assert (fix.frame_id, fix.child_frame_id) == ("world", "map")
        # The boundary rule under test: prior holds map_T_world = inv(world->map fix).
        np.testing.assert_allclose(
            reloc._fiducial_prior._fix_T, np.linalg.inv(fix.to_matrix()), atol=1e-9
        )
    finally:
        unsubscribe()
        vis.stop()
        reloc.stop()
