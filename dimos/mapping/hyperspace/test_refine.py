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

"""Each refinement step on a hand-built voxel map. No model, no store."""

from __future__ import annotations

import itertools

import numpy as np

from dimos.mapping.hyperspace import refine as rf
from dimos.mapping.hyperspace.patches import Heatmap

Index = tuple[int, int, int]


def block(lo: Index, hi: Index, score: float = 1.0) -> list[tuple[Index, float]]:
    """Every voxel of the closed box [lo, hi]."""
    return [
        ((x, y, z), score)
        for x, y, z in itertools.product(
            range(lo[0], hi[0] + 1), range(lo[1], hi[1] + 1), range(lo[2], hi[2] + 1)
        )
    ]


def heatmap(voxels: list[tuple[Index, float]], **extra: object) -> Heatmap:
    return Heatmap("odom", 0.1, sorted(voxels, key=lambda v: (-v[1], v[0])), {}, **extra)  # type: ignore[arg-type]


def indices(heat: Heatmap) -> set[Index]:
    return {i for i, _ in heat.voxels}


def test_closing_bridges_a_one_voxel_gap_and_opening_drops_a_speck() -> None:
    left = block((0, 0, 0), (2, 2, 2))
    right = block((4, 0, 0), (6, 2, 2))  # x = 3 is the gap
    speck = [((20, 20, 20), 1.0)]
    heat = heatmap(left + right + speck)
    config = rf.RefineConfig(closing_radius=1, opening_radius=1)
    closed = rf.closing(heat, config)
    assert (3, 1, 1) in indices(closed)  # the gap's core is bridged
    assert (20, 20, 20) in indices(closed)  # closing never removes
    joined = rf.components(closed, rf.RefineConfig(min_cluster=1))
    assert [c.voxels for c in joined.clusters][1:] == [1]  # blocks are one cluster, speck alone
    opened = indices(rf.opening(heat, config))
    assert (20, 20, 20) not in opened
    assert set(i for i, _ in left) <= opened


def test_gaussian_turns_speckle_into_one_blob_and_keeps_the_top_at_one() -> None:
    speckle = [((x, y, 0), 0.5) for x in range(0, 6, 2) for y in range(0, 6, 2)]  # a checkerboard
    result = rf.gaussian(heatmap(speckle), rf.RefineConfig(gaussian_sigma=1.0, cutoff=0.3))
    assert max(s for _, s in result.voxels) == 1.0
    assert (1, 1, 0) in indices(result)  # the holes filled in
    assert len(result.clusters) == 0  # gaussian alone labels nothing


def test_support_drops_single_view_voxels() -> None:
    voxels = [((0, 0, 0), 1.0), ((1, 0, 0), 0.9), ((2, 0, 0), 0.8)]
    support = {(0, 0, 0): (3, 2), (1, 0, 0): (1, 1), (2, 0, 0): (2, 1)}
    heat = heatmap(voxels, support=support)
    assert indices(rf.support(heat, rf.RefineConfig(min_frames=2, min_bins=1))) == {
        (0, 0, 0),
        (2, 0, 0),
    }
    assert indices(rf.support(heat, rf.RefineConfig(min_frames=2, min_bins=2))) == {(0, 0, 0)}


def test_occupancy_keeps_voxels_next_to_geometry_only() -> None:
    heat = heatmap([((0, 0, 0), 1.0), ((1, 0, 0), 1.0), ((9, 9, 9), 1.0)])
    scene = [(0, 0, 1)]  # right below the first voxel; one step from the second
    kept = indices(rf.occupancy(heat, rf.RefineConfig(occupancy_radius=1), scene))
    assert kept == {(0, 0, 0), (1, 0, 0)}


def test_prior_splits_a_row_of_chairs_and_slims_a_thick_door() -> None:
    # Twelve "chairs" fused into one 12 m long, 1 m wide slab.
    slab = block((0, 0, 0), (119, 9, 9))
    config = rf.RefineConfig(min_cluster=6)
    split = rf.components(rf.prior(heatmap(slab), config, rf.size_prior("a chair")), config)
    assert 11 <= len(split.clusters) <= 13, [c.extent for c in split.clusters]
    assert all(c.extent[0] <= 1.05 for c in split.clusters)
    # A 4 ft thick "door": 1.2 m wide, 1.2 m thick, 2 m tall; the prior keeps a 0.3 m face.
    door = block((0, 0, 0), (11, 11, 19))
    slim = rf.prior(heatmap(door), config, rf.size_prior("a door"))
    xs = {i[0] for i in indices(slim)}
    ys = {i[1] for i in indices(slim)}
    assert min(len(xs), len(ys)) <= 4  # <= 0.3 m across the thin axis (+1 voxel of slack)
    assert max(len(xs), len(ys)) >= 10  # the wide axis keeps its face (minus the margin)


def test_size_prior_lookup() -> None:
    assert rf.size_prior("a traffic cone") is rf.SIZE_PRIORS["cone"]
    assert rf.size_prior("the fire extinguisher") is rf.SIZE_PRIORS["fire extinguisher"]
    assert rf.size_prior("a zorblax") is rf.GENERIC_PRIOR


def test_components_rank_and_cut_clusters() -> None:
    big = block((0, 0, 0), (3, 3, 3), 0.9)  # 64 voxels
    small = block((10, 0, 0), (11, 1, 1), 1.0)  # 8 voxels
    speck = [((20, 0, 0), 1.0)]
    heat = heatmap(big + small + speck)
    result = rf.components(heat, rf.RefineConfig(min_cluster=6))
    assert [c.voxels for c in result.clusters] == [64, 8]
    assert result.clusters[0].rank == 0 and result.clusters[0].score == 1.0
    assert result.cluster_of[(10, 0, 0)] == 1 and (20, 0, 0) not in result.cluster_of
    assert np.allclose(result.clusters[1].centre, (1.1, 0.1, 0.1))
    assert result.clusters[0].extent == (0.4, 0.4, 0.4)
    top = rf.components(heat, rf.RefineConfig(min_cluster=6, top_k=1))
    assert len(top.clusters) == 1
    weak = rf.components(heat, rf.RefineConfig(min_cluster=1, min_ratio=0.5))
    assert [c.voxels for c in weak.clusters] == [64]


def test_refine_runs_the_chain_and_reports_it() -> None:
    heat = heatmap([*block((0, 0, 0), (2, 2, 2)), ((30, 30, 30), 1.0)])
    result = rf.refine(heat, rf.RefineConfig(methods=["opening"], min_cluster=1))
    assert result.stats["refine"] == ["opening", "components"]
    assert result.stats["clusters"] == 1
    assert rf.refine_config_of("default", "support,components").methods == ["support", "components"]  # type: ignore[union-attr]
    assert rf.refine_config_of("none", "support") is None
    assert rf.refine_config_of("default", "") is None


def test_structural_drops_floor_voxels_unless_the_query_is_about_the_floor() -> None:
    heat = heatmap([((0, 0, 0), 1.0), ((5, 5, 5), 1.0)])
    floor = [(0, 0, 1)]  # right under the first voxel
    config = rf.RefineConfig(structural_radius=1)
    assert indices(rf.structural(heat, config, floor, "a cone")) == {(5, 5, 5)}
    assert indices(rf.structural(heat, config, floor, "the floor")) == {(0, 0, 0), (5, 5, 5)}


def test_components_merge_the_same_object_placed_twice() -> None:
    near = block((0, 0, 0), (1, 1, 1), 1.0)  # 0.1 m cubes 0.3 m apart: one object
    twin = block((4, 0, 0), (5, 1, 1), 0.8)
    far = block((20, 0, 0), (21, 1, 1), 0.9)  # 2 m away: another
    heat = heatmap(near + twin + far)
    merged = rf.components(heat, rf.RefineConfig(min_cluster=1, merge_distance=0.6))
    assert [c.voxels for c in merged.clusters] == [16, 8]
    apart = rf.components(heat, rf.RefineConfig(min_cluster=1, merge_distance=0.0))
    assert len(apart.clusters) == 3
