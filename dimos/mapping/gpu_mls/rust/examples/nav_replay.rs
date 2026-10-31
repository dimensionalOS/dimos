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

//! The alfred-nav map path in one process: each frame goes through the ray
//! tracer's Mapper, then its local region into the MLS planner, as the two
//! modules do live, with alfred-nav's settings. Prints per-stage timings so
//! the CPU and GPU paths compare without a robot.
//! `both` runs the two in lockstep, then plans from the first pose to goals
//! every 2 m along the trajectory on each and compares which it reaches.
//! `cargo run --release --example nav_replay -- <frames> cpu|gpu|both [max_frames] [skip]`

use std::fs::File;
use std::io::{BufReader, Read};
use std::time::Instant;

use ahash::AHashSet;
use dimos_mls_planner::mls_planner::{Config as PlannerConfig, Planner, RegionBounds};
use dimos_voxel_ray_tracing::mapper::{Mapper, SurfaceRequest, SURFACE_CELL_FLOATS};
use dimos_voxel_ray_tracing::voxel_ray_tracer::Config;

type Point = (f32, f32, f32);

/// alfred-nav's map settings (_nav.py, alfred_nav.py, embodiment/alfred.py).
const VOXEL_SIZE_M: f32 = 0.08;
const MAX_RANGE_M: f32 = 15.0;
const ROBOT_HEIGHT_M: f32 = 1.86;
const WALL_CLEARANCE_M: f32 = 0.2;
const STEP_THRESHOLD_M: f32 = 0.08;
const GOAL_SPACING_M: f32 = 2.0;

struct Frames(BufReader<File>);

impl Iterator for Frames {
    type Item = (Point, Vec<Point>);

    fn next(&mut self) -> Option<Self::Item> {
        let mut head = [0u8; 16];
        self.0.read_exact(&mut head).ok()?;
        let f32_at = |b: &[u8], i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        let n = u32::from_le_bytes(head[0..4].try_into().unwrap()) as usize;
        let origin = (f32_at(&head, 4), f32_at(&head, 8), f32_at(&head, 12));
        let mut body = vec![0u8; 12 * n];
        self.0.read_exact(&mut body).expect("a whole frame");
        let points = (0..n)
            .map(|k| {
                (
                    f32_at(&body, 12 * k),
                    f32_at(&body, 12 * k + 4),
                    f32_at(&body, 12 * k + 8),
                )
            })
            .collect();
        Some((origin, points))
    }
}

fn mapper_config(gpu: bool) -> Config {
    Config {
        voxel_size: VOXEL_SIZE_M,
        fine_divisor: 3,
        emit_fine: false,
        max_range: MAX_RANGE_M,
        ray_subsample: 1,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        graze_cos: 0.7,
        graze_recent_frames: u32::MAX,
        gpu,
        gpu_max_points: 1 << 18,
        gpu_list_capacity: 1 << 20,
        support_min: 4,
        emit_every: 1,
        global_emit_every: 50,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        max_cloud_age_s: 0.0,
        tf_wait_timeout_s: 0.05,
        worker_threads: 4,
        region_m: 4.0,
        viz_cell_m: 4.0,
        viz_emit_every: 0,
        viz_sweep_cells: 0,
    }
}

fn planner_config(gpu: bool) -> PlannerConfig {
    PlannerConfig {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size: VOXEL_SIZE_M,
        robot_height: ROBOT_HEIGHT_M,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        wall_clearance_m: WALL_CLEARANCE_M,
        wall_buffer_m: 0.75,
        wall_buffer_weight: 100.0,
        step_threshold_m: STEP_THRESHOLD_M,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        gpu_surfaces: gpu,
        gpu_wall_distance: gpu,
        max_slope_deg: 45.0,
        surface_change_m: 0.01,
        worker_threads: 4,
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn summarize(name: &str, mut ms: Vec<f64>) {
    ms.sort_by(f64::total_cmp);
    println!(
        "{name:>10}: n {:5} median {:8.2} p95 {:8.2} max {:8.2} ms",
        ms.len(),
        percentile(&ms, 0.5),
        percentile(&ms, 0.95),
        ms.last().copied().unwrap_or(f64::NAN)
    );
}

/// One map path: the ray tracer's Mapper feeding the planner, and its timings.
struct Pipeline {
    gpu: bool,
    mapper: Mapper,
    planner: Planner,
    config: PlannerConfig,
    request: SurfaceRequest,
    frame_ms: Vec<f64>,
    surface_ms: Vec<f64>,
    planner_ms: Vec<f64>,
    total_ms: Vec<f64>,
    rows: Vec<f64>,
    dirty: Vec<f64>,
    bricks: Vec<f64>,
    region_bricks: Vec<f64>,
    extract_ms: Vec<f64>,
    export_ms: Vec<f64>,
    passes: std::collections::BTreeMap<&'static str, Vec<f64>>,
}

impl Pipeline {
    fn new(gpu: bool) -> Self {
        let mut mapper = Mapper::new(mapper_config(gpu));
        assert_eq!(mapper.on_gpu(), gpu, "the GPU map did not come up");
        if gpu && std::env::var("NAV_REPLAY_PASSES").is_ok() {
            assert!(mapper.time_gpu_passes(), "no GPU timestamps");
        }
        let config = planner_config(gpu);
        let request = SurfaceRequest {
            robot_height_m: config.robot_height,
            closing_radius_m: config.surface_closing_radius,
            step_threshold_m: config.step_threshold_m,
            max_slope_deg: config.max_slope_deg,
            wall_distance: config.gpu_wall_distance,
            wall_reach_m: config.wall_clearance_m + config.wall_buffer_m,
            max_overhead_m: config.max_overhead_m,
        };
        Self {
            gpu,
            mapper,
            planner: Planner::new(config.worker_threads),
            config,
            request,
            frame_ms: Vec::new(),
            surface_ms: Vec::new(),
            planner_ms: Vec::new(),
            total_ms: Vec::new(),
            rows: Vec::new(),
            dirty: Vec::new(),
            bricks: Vec::new(),
            region_bricks: Vec::new(),
            extract_ms: Vec::new(),
            export_ms: Vec::new(),
            passes: Default::default(),
        }
    }

    /// One frame through the mapper, then its region through the planner.
    fn step(&mut self, origin: Point, points: Vec<Point>) {
        let start = Instant::now();
        self.mapper.add_frame_world(points, origin);
        if !self.mapper.local_due() {
            return;
        }
        let cylinder = self.mapper.take_local_bounds();
        let local: Vec<Point> = self
            .mapper
            .local_points(&cylinder.bounds())
            .chunks_exact(3)
            .map(|p| (p[0], p[1], p[2]))
            .collect();
        let mapped = Instant::now();
        let cells = match self.gpu {
            true => self.mapper.surfaces(&cylinder, &self.request).map(|flat| {
                flat.chunks_exact(SURFACE_CELL_FLOATS)
                    .map(|c| (c[0], c[1], c[2], c[3], c[4]))
                    .collect::<Vec<_>>()
            }),
            false => None,
        };
        let surfaced = Instant::now();
        let region = RegionBounds {
            origin_x: cylinder.cx,
            origin_y: cylinder.cy,
            radius: cylinder.radius,
            z_min: cylinder.z_min,
            z_max: cylinder.z_max,
        }
        .capped_at(origin.2, self.config.max_overhead_m);
        match &cells {
            Some(cells) => {
                let update = self.mapper.surface_update();
                self.rows.push(cells.len() as f64);
                self.dirty.push(update.dirty_columns as f64);
                self.bricks.push(update.bricks as f64);
                self.region_bricks.push(update.region_bricks as f64);
                self.extract_ms.push(update.extract_ms);
                self.export_ms.push(update.export_ms);
                self.planner
                    .update_region_with_surface(&local, cells, &region, &self.config)
            }
            None => self.planner.update_region(&local, &region, &self.config),
        };
        let done = Instant::now();
        let mut per_pass: std::collections::BTreeMap<&'static str, f64> = Default::default();
        for (label, ms) in self.mapper.take_gpu_pass_times() {
            *per_pass.entry(label).or_default() += ms;
        }
        for (label, ms) in per_pass {
            self.passes.entry(label).or_default().push(ms);
        }
        let ms = |a: Instant, b: Instant| b.duration_since(a).as_secs_f64() * 1e3;
        self.frame_ms.push(ms(start, mapped));
        self.surface_ms.push(ms(mapped, surfaced));
        self.planner_ms.push(ms(surfaced, done));
        self.total_ms.push(ms(start, done));
    }

    fn report(&self) {
        println!(
            "{} {} regions, {} voxels and {} surface cells in the planner",
            if self.gpu { "GPU" } else { "CPU" },
            self.total_ms.len(),
            self.planner.voxel_count(),
            self.planner.surface().count()
        );
        summarize("frame", self.frame_ms.clone());
        summarize("surface", self.surface_ms.clone());
        summarize("planner", self.planner_ms.clone());
        summarize("total", self.total_ms.clone());
        if self.gpu {
            summarize("rows/upd", self.rows.clone());
            summarize("dirty cols", self.dirty.clone());
            summarize("bricks", self.bricks.clone());
            summarize("region brk", self.region_bricks.clone());
            summarize("extract", self.extract_ms.clone());
            summarize("export", self.export_ms.clone());
            for (label, ms) in &self.passes {
                summarize(label, ms.clone());
            }
        }
    }
}

/// The top of the floor under `o`, from the voxels in its column within 2 m.
fn floor_below(o: Point, occupied: &AHashSet<(i32, i32, i32)>) -> Option<f32> {
    let cell = |c: f32| (c / VOXEL_SIZE_M).floor() as i32;
    let (ix, iy, iz) = (cell(o.0), cell(o.1), cell(o.2));
    (1..(2.0 / VOXEL_SIZE_M) as i32)
        .find(|d| occupied.contains(&(ix, iy, iz - d)))
        .map(|d| (iz - d + 1) as f32 * VOXEL_SIZE_M)
}

/// Plan from the first floored pose to one every GOAL_SPACING_M on both.
fn compare_reach(cpu: &Pipeline, gpu: &Pipeline, origins: &[Point]) {
    let occupied: AHashSet<(i32, i32, i32)> = cpu.planner.voxel_keys().collect();
    let floored: Vec<(Point, f32)> = origins
        .iter()
        .filter_map(|&o| floor_below(o, &occupied).map(|f| (o, f)))
        .collect();
    let Some(&first) = floored.first() else {
        println!("no pose has a floor mapped below it");
        return;
    };
    let mut goals = vec![first];
    for &(o, f) in &floored {
        let (last, _) = goals[goals.len() - 1];
        let d = ((o.0 - last.0).powi(2) + (o.1 - last.1).powi(2) + (o.2 - last.2).powi(2)).sqrt();
        if d >= GOAL_SPACING_M {
            goals.push((o, f));
        }
    }
    let ground = |(o, f): (Point, f32)| (o.0, o.1, f + 0.5 * VOXEL_SIZE_M);
    let start = ground(goals[0]);
    let (mut both, mut only_cpu, mut only_gpu, mut neither) = (0, 0, 0, 0);
    for &g in &goals {
        let goal = ground(g);
        let c = cpu.planner.plan(start, goal, &cpu.config).is_some();
        let p = gpu.planner.plan(start, goal, &gpu.config).is_some();
        match (c, p) {
            (true, true) => both += 1,
            (true, false) => only_cpu += 1,
            (false, true) => only_gpu += 1,
            (false, false) => neither += 1,
        }
    }
    let n = goals.len() as f64;
    println!(
        "reach over {} goals: cpu {:.1}% gpu {:.1}% (both {both}, only cpu {only_cpu}, only gpu {only_gpu}, neither {neither})",
        goals.len(),
        100.0 * (both + only_cpu) as f64 / n,
        100.0 * (both + only_gpu) as f64 / n,
    );
}

fn main() {
    // NAV_REPLAY_LOG=debug prints the modules' stage timing lines.
    if let Ok(level) = std::env::var("NAV_REPLAY_LOG") {
        tracing_subscriber::fmt()
            .with_max_level(
                level
                    .parse::<tracing_subscriber::filter::LevelFilter>()
                    .expect("a log level"),
            )
            .with_writer(std::io::stderr)
            .init();
    }
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: nav_replay <frames> cpu|gpu|both [max_frames] [skip]";
    let path = args.get(1).expect(usage);
    let modes: &[bool] = match args.get(2).map(String::as_str) {
        Some("gpu") => &[true],
        Some("cpu") => &[false],
        Some("both") => &[false, true],
        _ => panic!("{usage}"),
    };
    let max_frames: usize = args.get(3).map_or(usize::MAX, |s| s.parse().expect(usage));
    let skip: usize = args.get(4).map_or(0, |s| s.parse().expect(usage));

    let mut pipelines: Vec<Pipeline> = modes.iter().map(|&gpu| Pipeline::new(gpu)).collect();
    let frames = Frames(BufReader::with_capacity(
        1 << 22,
        File::open(path).expect("a frames file"),
    ));
    let mut origins = Vec::new();
    for (i, (origin, points)) in frames.skip(skip).take(max_frames).enumerate() {
        origins.push(origin);
        for p in &mut pipelines {
            p.step(origin, points.clone());
        }
        if (i + 1) % 100 == 0 {
            eprintln!("frame {}", i + 1);
        }
    }
    for p in &pipelines {
        p.report();
    }
    if let [cpu, gpu] = pipelines.as_slice() {
        compare_reach(cpu, gpu, &origins);
    }
}
