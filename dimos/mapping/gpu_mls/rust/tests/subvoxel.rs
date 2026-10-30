// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Sub-voxel surfaces against geometric ground truth: step verdicts, ramps,
//! slopes, grid-line floors and headroom, at voxel sizes and grid offsets
//! where whole-voxel heights get them wrong.

mod common;

use ahash::{AHashMap, AHashSet};
use common::{cpu_config, gpu, gpu_config};
use dimos_gpu_mls::brick_map::{BrickMap, VoxelKey};
use dimos_gpu_mls::surfaces::{SurfaceConfig, SurfaceExtractor, Window};
use dimos_gpu_mls::synthetic::Lcg;

/// Jeff's examples use 7 cm voxels.
const VOXEL: f32 = 0.07;
/// Point spacing of the synthetic surfaces.
const SPACING: f32 = 0.01;
/// Uniform range noise, well under the margins the tests check.
const NOISE: f32 = 0.001;
/// Grid offsets each geometric verdict must hold at.
const OFFSETS: [f32; 6] = [0.0, 0.013, 0.027, 0.041, 0.055, 0.062];
/// A base height on a grid line, away from z = 0.
const BASE: f32 = 10.0 * VOXEL;
/// How close a plane height must come to the true surface.
const HEIGHT_TOLERANCE: f32 = 0.005;

type Cloud = Vec<(f32, f32, f32)>;

/// A surface z = height(x, y) sampled on a grid over the box, with noise.
fn patch(
    cloud: &mut Cloud,
    rng: &mut Lcg,
    (x0, x1): (f32, f32),
    (y0, y1): (f32, f32),
    height: impl Fn(f32, f32) -> f32,
) {
    let nx = ((x1 - x0) / SPACING).round() as i32;
    let ny = ((y1 - y0) / SPACING).round() as i32;
    for i in 0..=nx {
        for j in 0..=ny {
            let x = x0 + i as f32 * SPACING;
            let y = y0 + j as f32 * SPACING;
            cloud.push((x, y, height(x, y) + NOISE * (2.0 * rng.next() - 1.0)));
        }
    }
}

struct Surfaces {
    voxel: f32,
    /// Surface cell to its height and plane id.
    cells: AHashMap<VoxelKey, (f32, u32)>,
    standable: AHashSet<VoxelKey>,
    walls: AHashSet<VoxelKey>,
}

impl Surfaces {
    fn extract(cloud: &[(f32, f32, f32)], voxel: f32, config: SurfaceConfig) -> Self {
        let map_config = gpu_config(&cpu_config(voxel), 1 << 13, cloud.len() as u32);
        let mut map = BrickMap::new(gpu(), map_config).expect("brick map fits the device");
        map.seed(cloud);
        let keys = map
            .snapshot()
            .into_iter()
            .filter(|v| v.health > 0)
            .map(|v| v.key);
        let window =
            Window::covering(keys, config.closing_cells(voxel) + 1).expect("an occupied map");
        let mut extractor =
            SurfaceExtractor::new(&map, window, config).expect("window fits the device");
        extractor.extract(&mut map);
        map.finish();
        let result = extractor.read();
        let ids: AHashMap<VoxelKey, u32> = extractor.plane_ids().into_iter().collect();
        Self {
            voxel,
            cells: result
                .surface
                .iter()
                .zip(&result.heights)
                .map(|(&k, &h)| (k, (h, ids[&k])))
                .collect(),
            standable: result.standable.into_iter().collect(),
            walls: result.walls.into_iter().collect(),
        }
    }

    fn column(&self, x: f32, y: f32) -> (i32, i32) {
        (
            (x / self.voxel).floor() as i32,
            (y / self.voxel).floor() as i32,
        )
    }

    /// The surface cells of the column holding (x, y), lowest first.
    fn at(&self, x: f32, y: f32) -> Vec<(VoxelKey, f32, u32)> {
        let (cx, cy) = self.column(x, y);
        let mut out: Vec<_> = self
            .cells
            .iter()
            .filter(|(k, _)| k.0 == cx && k.1 == cy)
            .map(|(&k, &(h, id))| (k, h, id))
            .collect();
        out.sort_by_key(|c| c.0 .2);
        out
    }

    /// The single surface cell of a column.
    fn only(&self, x: f32, y: f32) -> (VoxelKey, f32, u32) {
        let cells = self.at(x, y);
        assert_eq!(
            cells.len(),
            1,
            "one surface cell at ({x}, {y}), got {cells:?}"
        );
        cells[0]
    }

    /// Whether the surfaces under two points are linked into one plane.
    fn linked(&self, a: (f32, f32), b: (f32, f32)) -> bool {
        self.only(a.0, a.1).2 == self.only(b.0, b.1).2
    }

    /// Columns whose centers fall inside a box.
    fn columns_inside(&self, (x0, x1): (f32, f32), (y0, y1): (f32, f32)) -> usize {
        let v = self.voxel;
        let count = |lo: f32, hi: f32| {
            ((lo / v - 0.5).floor() as i32 + 1..=(hi / v - 0.5).ceil() as i32 - 1).count()
        };
        count(x0, x1) * count(y0, y1)
    }

    /// Surface cells whose column centers fall inside a box.
    fn inside(&self, (x0, x1): (f32, f32), (y0, y1): (f32, f32)) -> Vec<VoxelKey> {
        let v = self.voxel;
        self.cells
            .keys()
            .copied()
            .filter(|k| {
                let (cx, cy) = ((k.0 as f32 + 0.5) * v, (k.1 as f32 + 0.5) * v);
                cx > x0 && cx < x1 && cy > y0 && cy < y1
            })
            .collect()
    }
}

fn config(step_threshold_m: f32) -> SurfaceConfig {
    SurfaceConfig {
        step_threshold_m,
        ..SurfaceConfig::mls_defaults()
    }
}

/// A step along x = 0: z = lower for x < 0, upper for x >= 0, with no riser points.
fn step_cloud(lower: f32, upper: f32) -> Cloud {
    let mut cloud = Cloud::new();
    let mut rng = Lcg(11);
    patch(&mut cloud, &mut rng, (-0.6, -0.005), (-0.5, 0.5), |_, _| {
        lower
    });
    patch(&mut cloud, &mut rng, (0.005, 0.6), (-0.5, 0.5), |_, _| {
        upper
    });
    cloud
}

/// The step verdict between the two columns either side of x = 0.
fn step_links(lower: f32, upper: f32, threshold: f32) -> bool {
    let s = Surfaces::extract(&step_cloud(lower, upper), VOXEL, config(threshold));
    for (x, truth) in [(-0.3, lower), (0.3, upper)] {
        let h = s.only(x, 0.0).1;
        assert!(
            (h - truth).abs() < HEIGHT_TOLERANCE,
            "height {h} where the surface is at {truth}"
        );
    }
    s.linked((-0.5 * VOXEL, 0.0), (0.5 * VOXEL, 0.0))
}

#[test]
fn one_cm_lip_across_a_grid_line_links() {
    // Near the top of one voxel next to near the bottom of the voxel above.
    let lower = BASE + 0.065;
    let upper = BASE + VOXEL + 0.005;
    assert!(step_links(lower, upper, 0.10));
}

#[test]
fn thirteen_cm_step_inside_two_voxels_follows_the_threshold() {
    // Near the bottom of one voxel next to near the top of the voxel above:
    // one voxel apart, so whole-voxel heights call it 7 cm.
    let lower = BASE + 0.005;
    let upper = BASE + VOXEL + 0.065;
    assert!(
        !step_links(lower, upper, 0.10),
        "a 13 cm step passes a 10 cm threshold"
    );
    assert!(
        step_links(lower, upper, 0.16),
        "a 13 cm step fails a 16 cm threshold"
    );
}

#[test]
fn step_verdicts_do_not_depend_on_the_grid_offset() {
    for offset in OFFSETS {
        let lower = BASE + offset;
        assert!(
            step_links(lower, lower + 0.15, 0.16),
            "15 cm step blocked at offset {offset}"
        );
        assert!(
            !step_links(lower, lower + 0.20, 0.16),
            "20 cm step linked at offset {offset}"
        );
    }
}

#[test]
fn ten_degree_ramp_is_one_surface_without_edges() {
    let slope = 10f32.to_radians().tan();
    let ramp = |x: f32, _: f32| BASE + slope * (x + 1.0);
    let mut cloud = Cloud::new();
    patch(&mut cloud, &mut Lcg(5), (-1.0, 1.0), (-0.5, 0.5), ramp);
    let s = Surfaces::extract(&cloud, VOXEL, SurfaceConfig::mls_defaults());
    let interior = s.inside((-0.85, 0.85), (-0.35, 0.35));
    assert_eq!(
        interior.len(),
        s.columns_inside((-0.85, 0.85), (-0.35, 0.35)),
        "one surface cell per ramp column"
    );
    let planes: AHashSet<u32> = interior.iter().map(|k| s.cells[k].1).collect();
    assert_eq!(
        planes.len(),
        1,
        "the ramp splits into {} surfaces",
        planes.len()
    );
    let edges = interior.iter().filter(|k| s.walls.contains(k)).count();
    assert_eq!(edges, 0, "{edges} fake edges along the ramp");
    for k in &interior {
        let x = (k.0 as f32 + 0.5) * VOXEL;
        let error = (s.cells[k].0 - ramp(x, 0.0)).abs();
        assert!(
            error < HEIGHT_TOLERANCE,
            "ramp height off by {error} at x {x}"
        );
    }
}

#[test]
fn sixty_degree_face_is_not_standable() {
    let slope = 60f32.to_radians().tan();
    let run = 0.3;
    let mut cloud = Cloud::new();
    let mut rng = Lcg(9);
    patch(&mut cloud, &mut rng, (-0.6, 0.0), (-0.5, 0.5), |_, _| BASE);
    patch(&mut cloud, &mut rng, (0.0, run), (-0.5, 0.5), |x, _| {
        BASE + slope * x
    });
    patch(&mut cloud, &mut rng, (run, 0.9), (-0.5, 0.5), |_, _| {
        BASE + slope * run
    });
    let face = ((VOXEL, run - VOXEL), (-0.35, 0.35));
    let gated = Surfaces::extract(&cloud, VOXEL, SurfaceConfig::mls_defaults());
    let on_face = gated.inside(face.0, face.1).len();
    assert_eq!(on_face, 0, "{on_face} standable cells on a 60 degree face");
    let ungated = Surfaces::extract(
        &cloud,
        VOXEL,
        SurfaceConfig {
            max_slope_deg: 90.0,
            ..SurfaceConfig::mls_defaults()
        },
    );
    assert!(
        !ungated.inside(face.0, face.1).is_empty(),
        "the face is only gated by the slope test"
    );
}

#[test]
fn floor_on_a_grid_line_is_one_surface() {
    for voxel in [0.05, VOXEL] {
        let height = 12.0 * voxel;
        let mut cloud = Cloud::new();
        patch(&mut cloud, &mut Lcg(3), (-0.6, 0.6), (-0.6, 0.6), |_, _| {
            height
        });
        let s = Surfaces::extract(&cloud, voxel, SurfaceConfig::mls_defaults());
        let interior = s.inside((-0.45, 0.45), (-0.45, 0.45));
        let mut per_column: AHashMap<(i32, i32), u32> = AHashMap::new();
        for k in &interior {
            *per_column.entry((k.0, k.1)).or_default() += 1;
            let error = (s.cells[k].0 - height).abs();
            assert!(
                error < HEIGHT_TOLERANCE,
                "height off by {error} at {voxel} m voxels"
            );
        }
        assert_eq!(
            per_column.len(),
            s.columns_inside((-0.45, 0.45), (-0.45, 0.45))
        );
        assert!(
            per_column.values().all(|&n| n == 1),
            "a column holds two surfaces at {voxel} m voxels"
        );
        let planes: AHashSet<u32> = interior.iter().map(|k| s.cells[k].1).collect();
        assert_eq!(planes.len(), 1);
        assert!(interior.iter().all(|k| !s.walls.contains(k)));
    }
}

#[test]
fn headroom_is_judged_in_meters() {
    let robot_height = SurfaceConfig::mls_defaults().robot_height_m;
    for voxel in [0.05, VOXEL] {
        for offset in OFFSETS {
            for (margin, clear) in [(0.02, true), (-0.02, false)] {
                let floor = BASE + offset;
                let ceiling = floor + robot_height + margin;
                let mut cloud = Cloud::new();
                let mut rng = Lcg(21);
                patch(&mut cloud, &mut rng, (-0.6, 0.6), (-0.6, 0.6), |_, _| floor);
                patch(&mut cloud, &mut rng, (-0.3, 0.3), (-0.3, 0.3), |_, _| {
                    ceiling
                });
                let s = Surfaces::extract(&cloud, voxel, SurfaceConfig::mls_defaults());
                let under: Vec<VoxelKey> = s
                    .inside((-0.2, 0.2), (-0.2, 0.2))
                    .into_iter()
                    .filter(|k| (s.cells[k].0 - floor).abs() < 0.02)
                    .collect();
                let columns = s.columns_inside((-0.2, 0.2), (-0.2, 0.2));
                let expected = if clear { columns } else { 0 };
                assert!(
                    under.len().abs_diff(expected) <= columns / 10,
                    "voxel {voxel} offset {offset} margin {margin}: {} of {columns} floor cells standable",
                    under.len()
                );
                assert!(s.standable.len() > 0);
            }
        }
    }
}
