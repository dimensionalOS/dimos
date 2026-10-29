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

//! The GPU brick map against the CPU VoxelMap on synthetic lidar frames.

mod common;

use common::{cpu_config, diff_maps, gpu, gpu_config};
use dimos_gpu_mls::brick_map::BrickMap;
use dimos_gpu_mls::synthetic::{Lcg, Scene};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{seed_points, update_map, Config, VoxelMap};

const FRAMES: usize = 8;
const POINTS_PER_FRAME: usize = 20_000;
const MOMENT_TOLERANCE: f64 = 1e-4;
const LONG_RUN_ROOM_M: f32 = 30.0;
const LONG_RUN_FRAMES: usize = 30;
/// Metal fuses `p - k * v` into one fma, so a point on a fine-cell boundary
/// can land in the neighboring fine cell. Allowed per voxel, as a fraction.
const FINE_TOLERANCE: f64 = 1e-4;

fn frames(scene: &Scene, cfg: &Config) -> Vec<((f32, f32, f32), Vec<(f32, f32, f32)>)> {
    let mut rng = Lcg(42);
    scene
        .trajectory(FRAMES, 0.4)
        .into_iter()
        .map(|o| {
            let pts = scene.scan(o, POINTS_PER_FRAME, &mut rng, 0.01, cfg.max_range + 5.0);
            (o, pts)
        })
        .collect()
}

#[test]
fn ingest_without_clearing_matches_cpu() {
    for voxel_size in [0.05, 0.08] {
        // A grace region longer than any ray and no shadow: the CPU walk clears nothing.
        let cfg = Config {
            grace_depth: 1e6,
            shadow_depth: 0.0,
            ..cpu_config(voxel_size)
        };
        let scene = Scene::room(12.0, 2.5);
        let mut cpu = VoxelMap::default();
        let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 16384, POINTS_PER_FRAME as u32))
            .expect("brick map fits the device");
        for (origin, pts) in frames(&scene, &cfg) {
            update_map(&mut cpu, origin, &pts, &cfg);
            map.ingest(origin, &pts);
        }
        map.finish();
        let d = diff_maps(&cpu, &map.snapshot(), voxel_size);
        eprintln!("ingest at {voxel_size}: {d:?}", d = summary(&d));
        assert_eq!(map.stats().overflow, 0);
        assert!(d.cpu_voxels > 10_000);
        assert_eq!(d.mismatched() - d.fine, 0, "{}", summary(&d));
        assert!(
            d.fine as f64 <= FINE_TOLERANCE * d.cpu_voxels as f64,
            "{}",
            summary(&d)
        );
        assert!(d.moment_rel_err < MOMENT_TOLERANCE, "{}", summary(&d));
    }
}

#[test]
fn ray_traced_clearing_matches_cpu() {
    for voxel_size in [0.05, 0.08] {
        let cfg = cpu_config(voxel_size);
        let scene = Scene::room(12.0, 2.5);
        let mut cpu = VoxelMap::default();
        let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 16384, POINTS_PER_FRAME as u32))
            .expect("brick map fits the device");
        let mut misses = 0;
        for (origin, pts) in frames(&scene, &cfg) {
            update_map(&mut cpu, origin, &pts, &cfg);
            map.integrate(origin, &pts);
            let stats = map.stats();
            assert_eq!(stats.overflow, 0);
            misses += stats.misses;
        }
        map.finish();
        let snapshot = map.snapshot();
        let d = diff_maps(&cpu, &snapshot, voxel_size);
        let fitted = snapshot.iter().filter(|v| !v.stale).count();
        eprintln!(
            "trace at {voxel_size}: {}; gpu misses {misses}, fitted normals {fitted}",
            summary(&d)
        );
        assert!(misses > 0, "the frames must exercise clearing");
        // Grazing-ray sparing hinges on f32 Jacobi fits and fused DDA math,
        // so boundary cases may flip; the rest must agree voxel for voxel.
        assert!(
            d.mismatched() as f64 <= 1e-3 * d.cpu_voxels as f64,
            "{}",
            summary(&d)
        );
        assert!(d.moment_rel_err < MOMENT_TOLERANCE, "{}", summary(&d));
    }
}

/// Every voxel a long run creates lies in the scene. On the Jetson a bad
/// voxel index once wrote fine-cell bits into brick coordinates, which
/// surfaced as keys millions of cells away and a garbage surface window.
#[test]
fn long_run_keys_stay_in_the_scene() {
    let cfg = cpu_config(0.05);
    let scene = Scene::room(LONG_RUN_ROOM_M, 3.0);
    let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 1 << 15, POINTS_PER_FRAME as u32))
        .expect("brick map fits the device");
    let mut rng = Lcg(3);
    let margin = 1.0;
    let lo = (-margin / cfg.voxel_size) as i32;
    let hi = ((LONG_RUN_ROOM_M + margin) / cfg.voxel_size) as i32;
    for (frame, origin) in scene
        .trajectory(LONG_RUN_FRAMES, 0.4)
        .into_iter()
        .enumerate()
    {
        let pts = scene.scan(
            origin,
            POINTS_PER_FRAME,
            &mut rng,
            0.01,
            cfg.max_range + 5.0,
        );
        map.integrate(origin, &pts);
        map.finish();
        let stray: Vec<_> = map
            .snapshot()
            .into_iter()
            .map(|v| v.key)
            .filter(|k| k.0 < lo || k.0 > hi || k.1 < lo || k.1 > hi || k.2 < lo || k.2 > hi)
            .collect();
        assert!(
            stray.is_empty(),
            "frame {frame}: {} voxels outside the scene, e.g. {:?}",
            stray.len(),
            &stray[..stray.len().min(4)]
        );
    }
}

#[test]
fn seed_matches_cpu() {
    let cfg = cpu_config(0.05);
    let scene = Scene::room(12.0, 2.5);
    let cloud: Vec<(f32, f32, f32)> = frames(&scene, &cfg)
        .into_iter()
        .flat_map(|(_, p)| p)
        .collect();
    let mut cpu = VoxelMap::default();
    seed_points(&mut cpu, &cloud, &cfg);
    let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 16384, cloud.len() as u32))
        .expect("brick map fits the device");
    map.seed(&cloud);
    // A second seed over existing voxels must change nothing.
    seed_points(&mut cpu, &cloud[..1000], &cfg);
    map.seed(&cloud[..1000]);
    map.finish();
    let d = diff_maps(&cpu, &map.snapshot(), cfg.voxel_size);
    eprintln!("seed: {}", summary(&d));
    assert_eq!(d.mismatched(), 0, "{}", summary(&d));
    assert!(d.moment_rel_err < MOMENT_TOLERANCE, "{}", summary(&d));
}

/// A floor and ceiling far bigger than the 65536-brick pool the map once had
/// fixed, seeded in pieces so the pools grow several times between frames,
/// then ray traced over: the grown map still matches the CPU.
#[test]
fn map_grows_past_the_old_fixed_pool_and_matches_cpu() {
    const OLD_POOL_BRICKS: u32 = 1 << 16;
    const ROOM_M: f32 = 110.0;
    const SPACING_M: f32 = 0.25;
    const SEED_PIECES: usize = 8;
    let cfg = cpu_config(0.05);
    let steps = (ROOM_M / SPACING_M) as usize;
    let cloud: Vec<(f32, f32, f32)> = [0.02f32, 2.48]
        .iter()
        .flat_map(|&z| {
            (0..steps * steps).map(move |i| {
                let (x, y) = ((i / steps) as f32, (i % steps) as f32);
                (x * SPACING_M + 0.01, y * SPACING_M + 0.01, z)
            })
        })
        .collect();
    let piece = cloud.len().div_ceil(SEED_PIECES);
    let mut cpu = VoxelMap::default();
    let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 1 << 19, piece as u32))
        .expect("brick map fits the device");
    for chunk in cloud.chunks(piece) {
        seed_points(&mut cpu, chunk, &cfg);
        map.seed(chunk);
    }
    let scene = Scene::room(ROOM_M, 2.5);
    let mut rng = Lcg(5);
    for origin in scene.trajectory(FRAMES, 0.4) {
        let pts = scene.scan(
            origin,
            POINTS_PER_FRAME,
            &mut rng,
            0.01,
            cfg.max_range + 5.0,
        );
        update_map(&mut cpu, origin, &pts, &cfg);
        map.integrate(origin, &pts);
    }
    map.finish();
    let stats = map.stats();
    assert_eq!(stats.overflow, 0);
    assert!(
        stats.bricks > OLD_POOL_BRICKS,
        "only {} bricks",
        stats.bricks
    );
    let d = diff_maps(&cpu, &map.snapshot(), cfg.voxel_size);
    eprintln!(
        "grown to {} bricks, {} voxels: {}",
        stats.bricks,
        stats.voxels,
        summary(&d)
    );
    assert!(
        d.mismatched() as f64 <= 1e-3 * d.cpu_voxels as f64,
        "{}",
        summary(&d)
    );
    assert!(d.moment_rel_err < MOMENT_TOLERANCE, "{}", summary(&d));
}

pub fn summary(d: &common::MapDiff) -> String {
    format!(
        "cpu {} gpu {} only_cpu {} only_gpu {} health {} count {} fine {} moment_rel_err {:.2e}",
        d.cpu_voxels,
        d.gpu_voxels,
        d.only_cpu.len(),
        d.only_gpu.len(),
        d.health,
        d.count,
        d.fine,
        d.moment_rel_err
    )
}

#[test]
fn full_pool_stops_taking_bricks_instead_of_filling_the_table() {
    let cfg = cpu_config(0.05);
    // A building-sized room: far more bricks than the pool or its table hold.
    let scene = Scene::room(60.0, 3.0);
    let mut rng = Lcg(7);
    // Small frames keep the table small (its headroom is one frame's points),
    // so new keys arriving after the pool filled would overflow it.
    let points = 2_000;
    let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 64, points as u32))
        .expect("brick map fits the device");
    for origin in scene.trajectory(40, 0.4) {
        let pts = scene.scan(origin, points, &mut rng, 0.01, cfg.max_range + 5.0);
        map.ingest(origin, &pts);
    }
    map.finish();
    let overflow = map.stats().overflow;
    assert_ne!(overflow & 512, 0, "the pool should report full");
    assert_eq!(
        overflow & 256,
        0,
        "a full pool must not fill the hash table"
    );
}
