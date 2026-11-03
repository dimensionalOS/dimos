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

//! A synthetic long run (no recording, no ICP): a robot circling a finite area for many hours with drifting odometry and
//! scans of a fixed point world; closures come from ground truth onto keyframes that still hold their clouds. Prints one
//! JSON line per report interval (keyframes, solve nodes, RSS, solve / pressure / scan-context time) and the final error.
//! usage: long_run <hours> [config overlay JSON] [--report-minutes M] [--seed S]

use std::sync::Arc;

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Vec3};

/// xorshift64*, for a run that is the same on every machine.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u, v) = (self.next().max(1e-300), self.next());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

fn rss_mb() -> f64 {
    let pid = std::process::id().to_string();
    std::process::Command::new("ps")
        .args(["-o", "rss=", "-p", &pid])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .and_then(|s| s.trim().parse::<f64>().ok())
        .map_or(f64::NAN, |kb| kb / 1024.0)
}

/// Live heap bytes (macOS malloc zones; the RSS also counts freed memory the allocator keeps); NaN elsewhere.
fn heap_mb() -> f64 {
    #[cfg(target_os = "macos")]
    {
        #[repr(C)]
        #[derive(Default)]
        struct MallocStatistics {
            blocks_in_use: u32,
            size_in_use: usize,
            max_size_in_use: usize,
            size_allocated: usize,
        }
        extern "C" {
            fn malloc_zone_statistics(zone: *mut std::ffi::c_void, stats: *mut MallocStatistics);
        }
        let mut stats = MallocStatistics::default();
        // SAFETY: a null zone asks for every zone's totals, written into `stats`
        unsafe { malloc_zone_statistics(std::ptr::null_mut(), &mut stats) };
        stats.size_in_use as f64 / (1024.0 * 1024.0)
    }
    #[cfg(not(target_os = "macos"))]
    f64::NAN
}

/// The route: corners of a few loops through a 40 m x 30 m area, driven in turn.
fn routes() -> Vec<Vec<[f64; 2]>> {
    vec![
        vec![[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]],
        vec![[0.0, 0.0], [20.0, 0.0], [20.0, 30.0], [40.0, 30.0], [40.0, 15.0], [0.0, 15.0]],
        vec![[0.0, 0.0], [0.0, 30.0], [40.0, 30.0], [40.0, 0.0]],
        vec![[0.0, 0.0], [10.0, 10.0], [30.0, 10.0], [30.0, 20.0], [10.0, 20.0], [0.0, 30.0], [0.0, 0.0]],
    ]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let hours: f64 = args.get(1).and_then(|a| a.parse().ok()).expect("usage: long_run <hours> [overlay]");
    let flag = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let report_s = flag("--report-minutes").map_or(30.0, |m| m.parse::<f64>().unwrap()) * 60.0;
    let mut rng = Rng(flag("--seed").map_or(0x9E3779B97F4A7C15, |s| s.parse().unwrap()));
    let mut config_json = serde_json::to_value(Config::default()).unwrap();
    if let Some(overlay) = args.get(2).filter(|a| a.starts_with('{')) {
        let overlay: serde_json::Value = serde_json::from_str(overlay).unwrap();
        for (k, v) in overlay.as_object().unwrap() {
            config_json[k] = v.clone();
        }
    }
    let config: Config = serde_json::from_value(config_json).unwrap();
    // the stated drift stands as the lidar range would set it
    let config = Config { odometry_error_estimate_from_lidar_range: false, ..config };
    let mut pgo = GscPgo::new(config.clone());
    pgo.set_wait_for_due_jobs(true);

    // the world: points on walls every 0.25 m around and through the area, binned in 5 m cells
    let mut world: std::collections::HashMap<(i64, i64), Vec<Vec3>> = Default::default();
    let mut wall = |a: [f64; 2], b: [f64; 2], rng: &mut Rng| {
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        for i in 0..(length / 0.25) as usize {
            let f = i as f64 * 0.25 / length;
            for h in 0..6 {
                let p = [
                    a[0] + f * (b[0] - a[0]) + 0.05 * rng.normal(),
                    a[1] + f * (b[1] - a[1]) + 0.05 * rng.normal(),
                    h as f64 * 0.4,
                ];
                world.entry(((p[0] / 5.0).floor() as i64, (p[1] / 5.0).floor() as i64)).or_default().push(p);
            }
        }
    };
    for (a, b) in [
        ([-6.0, -6.0], [46.0, -6.0]),
        ([46.0, -6.0], [46.0, 36.0]),
        ([46.0, 36.0], [-6.0, 36.0]),
        ([-6.0, 36.0], [-6.0, -6.0]),
    ] {
        wall(a, b, &mut rng);
    }
    for _ in 0..40 {
        let (x, y) = (rng.next() * 40.0, rng.next() * 30.0);
        let (dx, dy) = (rng.normal() * 2.0, rng.normal() * 2.0);
        wall([x, y], [x + dx, y + dy], &mut rng);
    }

    let (speed, rate) = (0.6, 10.0);
    let routes = routes();
    let (mut route, mut leg) = (0usize, 0usize);
    let mut gt = [0.0f64, 0.0, 0.0]; // x, y, yaw
    let (mut odom_xy, mut odom_yaw) = ([0.0f64, 0.0], 0.0f64);
    let mut keyframe_truth: Vec<[f64; 3]> = Vec::new();
    let mut time = 1.0e9;
    let total_scans = (hours * 3600.0 * rate) as usize;
    let started = std::time::Instant::now();
    let mut last_report = (0.0f64, 0usize, 0usize, 0usize, [0.0f64; 10]);
    let (mut peak_rss, mut peak_heap) = (0.0f64, 0.0f64);
    for scan in 0..total_scans {
        // drive toward the next corner, turning at up to 0.8 rad/s
        let corner = routes[route][(leg + 1) % routes[route].len()];
        let (dx, dy) = (corner[0] - gt[0], corner[1] - gt[1]);
        if dx.hypot(dy) < 0.5 {
            leg += 1;
            if leg >= routes[route].len() {
                (route, leg) = ((route + 1) % routes.len(), 0);
            }
        }
        let want = dy.atan2(dx);
        let turn = (want - gt[2] + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
        let turn = turn.clamp(-0.8 / rate, 0.8 / rate);
        let step = speed / rate;
        gt[2] += turn;
        gt[0] += step * gt[2].cos();
        gt[1] += step * gt[2].sin();
        // odometry: the same motion with a yaw bias and random walks (several times the stated estimate)
        odom_yaw += turn * (1.0 + 0.002) + 0.0005 * step.sqrt() * rng.normal();
        let forward = step * (1.0 + 0.001 * rng.normal());
        odom_xy[0] += forward * odom_yaw.cos();
        odom_xy[1] += forward * odom_yaw.sin();
        time += 1.0 / rate;

        // the scan: world points within 15 m in the body frame, jittered
        let (cos, sin) = (gt[2].cos(), gt[2].sin());
        let mut cloud = Vec::new();
        let cell = ((gt[0] / 5.0).floor() as i64, (gt[1] / 5.0).floor() as i64);
        for cx in -3..=3 {
            for cy in -3..=3 {
                for p in world.get(&(cell.0 + cx, cell.1 + cy)).into_iter().flatten() {
                    let (rx, ry) = (p[0] - gt[0], p[1] - gt[1]);
                    if rx.hypot(ry) < 15.0 && rng.next() < 0.3 {
                        cloud.push([
                            (cos * rx + sin * ry + 0.01 * rng.normal()) as f32,
                            (-sin * rx + cos * ry + 0.01 * rng.normal()) as f32,
                            (p[2] - 0.5) as f32,
                        ]);
                    }
                }
            }
        }
        let mut pose = PoseWithTime::new(mat3::rot_z(odom_yaw), [odom_xy[0], odom_xy[1], 0.0]);
        pose.set_time(time as i32, ((time.fract()) * 1e9) as u32);
        if pgo.add_key_pose(&CloudWithPose { cloud: Some(Arc::new(cloud)), pose }) {
            keyframe_truth.push(gt);
            let n = pgo.key_poses().len();
            // scan-context lookup as the search would do it (the ICP half is not run)
            pgo.propose_closure();
            drop(pgo.begin_closure_search());
            // every 10th keyframe: a closure from truth onto the nearest older keyframe that still holds its cloud
            if n.is_multiple_of(10) && n > 100 {
                let here = keyframe_truth[n - 1];
                let target = (0..n - 60)
                    .filter(|&k| !pgo.key_poses().is_removed(k))
                    .map(|k| (((keyframe_truth[k][0] - here[0]).hypot(keyframe_truth[k][1] - here[1])), k))
                    .filter(|&(d, _)| d < 2.0)
                    .min_by(|a, b| a.partial_cmp(b).unwrap());
                if let Some((_, target)) = target {
                    // relative pose from truth (yaw only), source in the target's frame, plus ICP-sized noise
                    let (t, s) = (keyframe_truth[target], here);
                    let (t_yaw, s_yaw) = (t[2], s[2]);
                    let (c, si) = (t_yaw.cos(), t_yaw.sin());
                    let (rx, ry) = (s[0] - t[0], s[1] - t[1]);
                    let outlier = rng.next() < 0.02;
                    let noise = if outlier { 3.0 } else { 0.02 };
                    pgo.commit_closure(
                        n - 1,
                        target,
                        mat3::rot_z(s_yaw - t_yaw + 0.003 * rng.normal()),
                        [
                            c * rx + si * ry + noise * rng.normal(),
                            -si * rx + c * ry + noise * rng.normal(),
                            0.0,
                        ],
                        0.01,
                    );
                }
            }
        }
        pgo.wait_for_due_solve();
        let data_s = (scan + 1) as f64 / rate;
        if data_s - last_report.0 >= report_s || scan + 1 == total_scans {
            let rss = rss_mb();
            peak_rss = peak_rss.max(rss);
            peak_heap = peak_heap.max(heap_mb());
            let solves = &pgo.solve_records()[last_report.1..];
            let pressure = &pgo.pressure_calls()[last_report.2..];
            let stages = pgo.stage_seconds();
            let stats = pgo.simplify_stats();
            let mean = |v: &mut dyn Iterator<Item = f64>| {
                let (s, c) = v.fold((0.0, 0usize), |(s, c), x| (s + x, c + 1));
                if c > 0 { s / c as f64 } else { f64::NAN }
            };
            let line = serde_json::json!({
                "hours": data_s / 3600.0,
                "keyframes": pgo.key_poses().len(),
                "nodes": stats.nodes,
                "removed": stats.removed,
                "folded_edges": stats.folded_edges,
                "solve_nodes": stats.solve_nodes,
                "solve_closures": stats.solve_closures,
                "awaiting_release": stats.awaiting_release,
                "closures": pgo.committed_closures().len(),
                "rss_mb": rss,
                "heap_mb": heap_mb(),
                "solves": solves.len(),
                "solve_mean_s": mean(&mut solves.iter().map(|s| s.2)),
                "solve_max_s": solves.iter().map(|s| s.2).fold(f64::NAN, f64::max),
                "pressure_calls": pressure.len(),
                "pressure_mean_ms": mean(&mut pressure.iter().map(|p| p.2 * 1e3)),
                "scan_context_ms_per_keyframe": (stages[4].1 - last_report.4[4]) * 1e3
                    / (pgo.key_poses().len() - last_report.3).max(1) as f64,
                "wall_s": started.elapsed().as_secs_f64(),
            });
            println!("{line}");
            last_report = (
                data_s,
                pgo.solve_records().len(),
                pgo.pressure_calls().len(),
                pgo.key_poses().len(),
                stages.map(|s| s.1),
            );
        }
    }
    pgo.finish();
    let n = pgo.key_poses().len();
    let errors: Vec<f64> = (0..n)
        .map(|k| {
            let p = pgo.key_poses().position(k);
            (p[0] - keyframe_truth[k][0]).hypot(p[1] - keyframe_truth[k][1])
        })
        .collect();
    let mut sorted = errors.clone();
    sorted.sort_by(f64::total_cmp);
    println!(
        "{}",
        serde_json::json!({
            "final": true,
            "keyframes": n,
            "nodes": pgo.simplify_stats().nodes,
            "ate_rms_m": (errors.iter().map(|e| e * e).sum::<f64>() / n as f64).sqrt(),
            "ate_median_m": sorted[n / 2],
            "ate_max_m": sorted[n - 1],
            "kept_closures": pgo.committed_closures().iter().filter(|c| c.gnc_weight > 0.5).count(),
            "peak_rss_mb": peak_rss,
            "peak_heap_mb": peak_heap,
            "wall_s": started.elapsed().as_secs_f64(),
        })
    );
}
