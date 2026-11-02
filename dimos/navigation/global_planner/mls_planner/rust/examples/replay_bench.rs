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

//! Replay exported lidar frames through the ray tracer and the MLS planner in
//! lockstep, the way their modules run live, then score the final graph.
//!
//!     cargo run --release --example replay_bench -- <frames> <configs.json> <out.json> \
//!         [--stair-frames FIRST LAST] [--frames N] [--dump DIR]
//!
//! `frames` comes from utils/export_frames.py and `configs.json` from
//! utils/bench_configs.py. The score has per-frame latency, reach to goals
//! spaced along the trajectory, each leg's path against its trajectory length,
//! and, with --stair-frames, how the frames of one climb connect. --dump
//! writes the final map for the viewers: `voxels.f32`, every voxel center
//! and plane as the tracer emits it, `cells.f32`, every surface cell's
//! center, surface height, and wall clearance, and `meta.json`, the voxel size.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufReader, Read};
use std::time::Instant;

use dimos_mls_planner::mls_planner::{Config as PlannerConfig, Planner, RegionBounds};
use dimos_mls_planner::module::extract_points;
use dimos_voxel_ray_tracing::mapper::{Mapper, Pose};
use dimos_voxel_ray_tracing::module::surfaces_to_cloud;
use dimos_voxel_ray_tracing::voxel_ray_tracer::{Config as TracerConfig, LocalBounds};
use lcm_msgs::std_msgs::Time;
use serde::Deserialize;
use serde_json::json;

type Xyz = (f32, f32, f32);

#[derive(Deserialize)]
struct BenchConfigs {
    tracer: TracerConfig,
    planner: PlannerConfig,
    /// Height of the lidar above the ground the robot stands on.
    sensor_height_m: f32,
    /// Trajectory distance between consecutive goals.
    goal_spacing_m: f32,
    /// Leg paths longer than this multiple of their trajectory length are detours.
    detour_ratio: f32,
    /// Half width of the corridor along the stair frames whose cells are scored.
    stair_corridor_m: f32,
}

struct Frame {
    stamp: f64,
    pose: Pose,
    points: Vec<Xyz>,
}

fn read_frame(r: &mut impl Read) -> Option<Frame> {
    let mut head = [0u8; 8 + 7 * 4 + 4];
    r.read_exact(&mut head).ok()?;
    let f = |i: usize| f32::from_le_bytes(head[8 + 4 * i..12 + 4 * i].try_into().unwrap());
    let n = u32::from_le_bytes(head[36..40].try_into().unwrap()) as usize;
    let mut body = vec![0u8; n * 12];
    r.read_exact(&mut body).ok()?;
    let points = body
        .as_chunks::<12>()
        .0
        .iter()
        .map(|p| {
            let c = |k: usize| f32::from_le_bytes(p[4 * k..4 * k + 4].try_into().unwrap());
            (c(0), c(1), c(2))
        })
        .collect();
    Some(Frame {
        stamp: f64::from_le_bytes(head[0..8].try_into().unwrap()),
        pose: Pose {
            position: (f(0), f(1), f(2)),
            orientation: (f(3), f(4), f(5), f(6)),
        },
        points,
    })
}

fn stamp_of(t: f64) -> Time {
    Time {
        sec: t.floor() as i32,
        nsec: ((t - t.floor()) * 1e9) as i32,
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn dist(a: Xyz, b: Xyz) -> f32 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt()
}

fn path_length(path: &[Xyz]) -> f32 {
    path.windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// Cells that keep the hard wall clearance, reachable from `from` over edges.
fn passable_component(planner: &Planner, from: u32, clearance: f32) -> Vec<bool> {
    let graph = planner.graph();
    let wall_dist = &graph.wall_state.dist;
    let passable = |id: u32| wall_dist.get(id as usize).is_some_and(|&d| d >= clearance);
    let mut seen = vec![false; graph.cells.slot_capacity()];
    if !passable(from) {
        return seen;
    }
    let mut queue = VecDeque::from([from]);
    seen[from as usize] = true;
    while let Some(id) = queue.pop_front() {
        for e in graph.cells.neighbors(id) {
            if !seen[e.dest as usize] && passable(e.dest) && e.cost.is_finite() {
                seen[e.dest as usize] = true;
                queue.push_back(e.dest);
            }
        }
    }
    seen
}

/// The surface cell in the ground point's column nearest it in z, if any lies
/// within the robot height.
fn cell_under(planner: &Planner, ground: Xyz, config: &PlannerConfig) -> Option<u32> {
    let v = config.voxel_size;
    let col = ((ground.0 / v).floor() as i32, (ground.1 / v).floor() as i32);
    let graph = planner.graph();
    graph
        .surface_near(col, ground.2)
        .filter(|&id| (graph.cells.z(id) - ground.2).abs() <= config.robot_height)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage = "replay_bench <frames> <configs.json> <out.json> [--stair-frames FIRST LAST] [--frames N] [--dump DIR]";
    let [_, frames_path, configs_path, out_path, rest @ ..] = args.as_slice() else {
        panic!("{usage}");
    };
    let mut stair: Option<(usize, usize)> = None;
    let mut max_frames = usize::MAX;
    let mut dump: Option<std::path::PathBuf> = None;
    let mut it = rest.iter();
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--stair-frames" => {
                let mut value = || it.next().expect(usage).parse::<usize>().expect(usage);
                stair = Some((value(), value()));
            }
            "--frames" => max_frames = it.next().expect(usage).parse().expect(usage),
            "--dump" => dump = Some(it.next().expect(usage).into()),
            _ => panic!("{usage}"),
        }
    }
    let configs: BenchConfigs =
        serde_json::from_reader(File::open(configs_path).expect("configs")).expect("configs json");
    let (tracer_config, config) = (configs.tracer, configs.planner);
    let world_frame = tracer_config.world_frame.clone();
    let mut mapper = Mapper::new(tracer_config);
    let mut planner = Planner::new(config.worker_threads);

    let mut reader = BufReader::new(File::open(frames_path).expect("frames"));
    let mut tracer_ms: Vec<f64> = Vec::new();
    let mut planner_ms: Vec<f64> = Vec::new();
    let mut total_ms: Vec<f64> = Vec::new();
    let mut ground: Vec<Xyz> = Vec::new();
    while ground.len() < max_frames {
        let Some(frame) = read_frame(&mut reader) else {
            break;
        };
        let (sx, sy, sz) = frame.pose.position;
        ground.push((sx, sy, sz - configs.sensor_height_m));

        let start = Instant::now();
        mapper.add_frame(frame.points, frame.pose);
        let local = mapper.local_due().then(|| {
            let cylinder = mapper.take_local_bounds();
            let cloud = surfaces_to_cloud(
                &mapper.local_surfaces(&cylinder.bounds()),
                &world_frame,
                stamp_of(frame.stamp),
            );
            (cylinder, cloud)
        });
        let traced = start.elapsed().as_secs_f64() * 1e3;
        let Some((cylinder, cloud)) = local else {
            continue;
        };
        let start = Instant::now();
        let points = extract_points(&cloud).unwrap_or_else(|e| panic!("local_map: {e}"));
        let region = RegionBounds {
            origin_x: cylinder.cx,
            origin_y: cylinder.cy,
            radius: cylinder.radius,
            z_min: cylinder.z_min,
            z_max: cylinder.z_max,
        }
        .capped_at(sz, config.max_overhead_m);
        planner.update_region(&points, &region, &config);
        let planned = start.elapsed().as_secs_f64() * 1e3;
        tracer_ms.push(traced);
        planner_ms.push(planned);
        total_ms.push(traced + planned);
    }

    // Goals every goal_spacing_m of trajectory, each leg planned from the last goal.
    let mut along = 0.0_f32;
    let mut goal_frames = vec![0usize];
    for i in 1..ground.len() {
        along += dist(ground[i - 1], ground[i]);
        if along >= configs.goal_spacing_m {
            goal_frames.push(i);
            along = 0.0;
        }
    }
    let mut goals = Vec::new();
    let mut reached = 0usize;
    let mut detours = 0usize;
    for w in goal_frames.windows(2) {
        let (from, to) = (w[0], w[1]);
        let trajectory: f32 = ground[from..=to].windows(2).map(|p| dist(p[0], p[1])).sum();
        let from_start = planner.plan(ground[0], ground[to], &config);
        let leg = planner.plan(ground[from], ground[to], &config);
        let leg_m = leg.as_deref().map(path_length);
        reached += from_start.is_some() as usize;
        let detour = leg_m.is_none_or(|m| m > configs.detour_ratio * trajectory);
        detours += detour as usize;
        goals.push(json!({
            "frame": to,
            "goal": [ground[to].0, ground[to].1, ground[to].2],
            "reached_from_start": from_start.is_some(),
            "leg_m": leg_m,
            "trajectory_m": trajectory,
            "detour": detour,
            "leg_path": leg,
        }));
    }

    let stair_score = stair.map(|(first, last)| {
        let clearance = config.wall_clearance_m;
        let first_cell = cell_under(&planner, ground[first], &config);
        let component =
            first_cell.map_or_else(Vec::new, |id| passable_component(&planner, id, clearance));
        let frames: Vec<bool> = (first..=last)
            .map(|i| {
                cell_under(&planner, ground[i], &config)
                    .is_some_and(|id| component.get(id as usize).copied().unwrap_or(false))
            })
            .collect();
        let wall_dist = &planner.graph().wall_state.dist;
        let blocked = (first..=last)
            .filter_map(|i| cell_under(&planner, ground[i], &config))
            .filter(|&id| wall_dist.get(id as usize).is_none_or(|&d| d < clearance))
            .count();
        // Surface cells within the corridor of the climb, at its height there.
        let graph = planner.graph();
        let v = config.voxel_size;
        let corridor: Vec<u32> = graph
            .cells
            .ids()
            .filter(|&id| {
                let (x, y, z) = graph.cells.point(id, v);
                ground[first..=last].iter().any(|g| {
                    (x - g.0).hypot(y - g.1) <= configs.stair_corridor_m
                        && (z - g.2).abs() <= config.step_threshold_m
                })
            })
            .collect();
        let corridor_blocked = corridor
            .iter()
            .filter(|&&id| wall_dist.get(id as usize).is_none_or(|&d| d < clearance))
            .count();
        json!({
            "first": first,
            "last": last,
            "corridor_cells": corridor.len(),
            "corridor_under_clearance": corridor_blocked,
            "component_cells": component.iter().filter(|&&s| s).count(),
            "frames_connected": frames.iter().filter(|&&c| c).count(),
            "frames": frames.len(),
            "frames_under_clearance": blocked,
            "first_disconnected": frames.iter().position(|&c| !c).map(|k| first + k),
        })
    });

    if let Some(dir) = dump {
        std::fs::create_dir_all(&dir).expect("dump dir");
        // Every voxel lies within the tracer's range of some sensor pose.
        let n = ground.len() as f32;
        let (cx, cy) = ground
            .iter()
            .fold((0.0, 0.0), |(x, y), g| (x + g.0 / n, y + g.1 / n));
        let reach = mapper.config().max_range;
        let far = ground
            .iter()
            .map(|g| (g.0 - cx).hypot(g.1 - cy))
            .fold(0.0, f32::max)
            + reach;
        let (lo, hi) = ground.iter().fold((f32::MAX, f32::MIN), |(lo, hi), g| {
            (lo.min(g.2), hi.max(g.2))
        });
        let everywhere = LocalBounds {
            origin_x: cx,
            origin_y: cy,
            r_xy_max_sq: far * far,
            z_min: lo - reach,
            z_max: hi + reach,
        };
        let voxels = mapper.local_surfaces(&everywhere);
        let v = config.voxel_size;
        let cells: Vec<f32> = planner
            .surface_clearance()
            .into_iter()
            .flat_map(|((ix, iy, _), z, clearance)| {
                [(ix as f32 + 0.5) * v, (iy as f32 + 0.5) * v, z, clearance]
            })
            .collect();
        let bytes =
            |values: &[f32]| -> Vec<u8> { values.iter().flat_map(|f| f.to_le_bytes()).collect() };
        std::fs::write(dir.join("voxels.f32"), bytes(&voxels)).expect("write voxels");
        std::fs::write(dir.join("cells.f32"), bytes(&cells)).expect("write cells");
        let meta = json!({ "voxel_size": v });
        std::fs::write(dir.join("meta.json"), meta.to_string()).expect("write meta");
    }

    let mut sorted = total_ms.clone();
    sorted.sort_by(f64::total_cmp);
    let median = |v: &[f64]| {
        let mut sorted = v.to_vec();
        sorted.sort_by(f64::total_cmp);
        percentile(&sorted, 0.5)
    };
    let out = json!({
        "frames": ground.len(),
        "regions": total_ms.len(),
        "latency_ms": {
            "median": percentile(&sorted, 0.5),
            "p95": percentile(&sorted, 0.95),
            "tracer_median": median(&tracer_ms),
            "planner_median": median(&planner_ms),
        },
        "voxels": planner.voxel_count(),
        "surface_cells": planner.graph().cells.ids().count(),
        "goals": goals.len(),
        "reached_from_start": reached,
        "leg_detours": detours,
        "stair": stair_score,
        "goal_detail": goals,
    });
    std::fs::write(out_path, serde_json::to_string_pretty(&out).unwrap()).expect("write out");
    let summary = json!({"latency_ms": out["latency_ms"], "goals": out["goals"],
        "reached_from_start": reached, "leg_detours": detours, "stair": out["stair"]});
    println!("{}", serde_json::to_string_pretty(&summary).unwrap());
}
