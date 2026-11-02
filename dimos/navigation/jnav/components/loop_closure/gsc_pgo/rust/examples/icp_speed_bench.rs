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

//! Time the single closure ICP's speed options on dumped candidates (`closure_icp_dump_dir`, single-scan files only):
//! each candidate's ICP is re-run from its real initial guess (version 2 dumps; a version 1 dump has none, so it starts a
//! metre off its dumped result), as is, with the early stop, and with the target thinned, and compared with the as-is
//! result (pass/fail). With stems given, each one's every variant is printed.
//! usage: icp_speed_bench <dump dir> [max files] [per-candidate csv out] [stem,stem,... e.g. 3014_27]
use dimos_gsc_pgo::connection_fit::{icp_thinned, IcpParams, IcpResult, ICP_DUMP_VERSION};
use dimos_gsc_pgo::gsc_pgo::Config;
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pointcloud::KdTree;
use std::time::Instant;

/// A dump's source and target clouds, its initial guess and whether that guess is the real one.
type Dump = (Vec<[f32; 3]>, Vec<[f32; 3]>, (Mat3, Vec3), bool);

/// Reads a dump (see `Dump`).
fn read_dump(bytes: &[u8]) -> Dump {
    let ns = u32::from_le_bytes(bytes[0..4].try_into().unwrap()) as usize;
    let nt = u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize;
    let points: Vec<[f32; 3]> = bytes[8..8 + 12 * (ns + nt)]
        .as_chunks::<12>()
        .0
        .iter()
        .map(|c| [0, 4, 8].map(|i| f32::from_le_bytes(c[i..i + 4].try_into().unwrap())))
        .collect();
    let tail = &bytes[8 + 12 * (ns + nt)..];
    let f = |at: usize| f64::from_le_bytes(tail[at..at + 8].try_into().unwrap());
    let pose = |at: usize| -> (Mat3, Vec3) {
        (
            std::array::from_fn(|i| std::array::from_fn(|j| f(at + 8 * (3 * i + j)))),
            [f(at + 72), f(at + 80), f(at + 88)],
        )
    };
    // version 1: result pose (12 f64), fitness, converged; version 2 adds the guess (12 f64), iterations, stop, version
    let version = if tail.len() >= 105 + 96 + 9 {
        u32::from_le_bytes(tail[tail.len() - 4..].try_into().unwrap())
    } else {
        1
    };
    let (guess, real) = if version >= ICP_DUMP_VERSION {
        (pose(105), true)
    } else {
        let (rotation, translation) = pose(0);
        (
            (rotation, mat3::add(&translation, &[1.0, -0.5, 0.0])),
            false,
        )
    };
    let (source, target) = points.split_at(ns);
    (source.to_vec(), target.to_vec(), guess, real)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let max: usize = args.get(2).map_or(200, |s| s.parse().unwrap());
    let stems: Vec<String> = args
        .get(4)
        .map_or(Vec::new(), |s| s.split(',').map(String::from).collect());
    let config = Config::default();
    let max_fitness = config.closure_icp_max_fitness_m.powi(2);
    let params = IcpParams {
        max_iterations: config.closure_icp_max_iterations as usize,
        max_correspondence_distance: config.closure_icp_max_correspondence_m,
        inlier_squared_distance: config.closure_inlier_squared_distance(),
        point_to_plane: config.closure_icp_point_to_plane,
    };
    let mut files: Vec<_> = std::fs::read_dir(&args[1])
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            let stem = p.file_stem().unwrap().to_string_lossy().to_string();
            stem.split('_').count() == 2 && (stems.is_empty() || stems.contains(&stem))
        })
        .collect();
    files.sort();
    let resolution = config.closure_icp_target_resolution_m;
    // (name, early stop, target resolution: 0 = full)
    let variants: Vec<(&str, bool, f64)> = vec![
        ("as is", false, 0.0),
        ("early stop", true, 0.0),
        ("thin", false, resolution),
        ("early + thin (default)", true, resolution),
    ];
    let k = variants.len();
    let mut time = vec![0.0f64; k];
    let mut flips = vec![0usize; k];
    let mut passes = vec![0usize; k];
    let (mut n, mut real_guesses) = (0usize, 0usize);
    let mut csv = String::from("file,variant,pass,fitness,iterations,stop,tx,ty,tz\n");
    for path in files.iter().take(max) {
        let (source, target, (rotation, translation), real) =
            read_dump(&std::fs::read(path).unwrap());
        real_guesses += real as usize;
        let tree = KdTree::build(&target);
        let pass = |r: &IcpResult| r.converged && r.fitness <= max_fitness;
        let mut base = false;
        for (v, &(name, early, resolution)) in variants.iter().enumerate() {
            let t0 = Instant::now();
            let r = icp_thinned(
                &source,
                &target,
                &tree,
                &rotation,
                &translation,
                &params,
                early.then_some(max_fitness),
                resolution,
            );
            time[v] += t0.elapsed().as_secs_f64();
            if v == 0 {
                base = pass(&r);
            }
            flips[v] += (pass(&r) != base) as usize;
            passes[v] += pass(&r) as usize;
            let file = path.file_name().unwrap().to_string_lossy();
            if !stems.is_empty() {
                println!(
                    "{file} {name:<24} pass {} fitness {:.4} iterations {:3} {}",
                    pass(&r) as u8,
                    r.fitness,
                    r.iterations,
                    r.stop.name()
                );
            }
            csv += &format!(
                "{file},{name},{},{},{},{},{},{},{}\n",
                pass(&r) as u8,
                r.fitness,
                r.iterations,
                r.stop.name(),
                r.translation[0],
                r.translation[1],
                r.translation[2]
            );
        }
        n += 1;
    }
    if let Some(out) = args.get(3).filter(|s| !s.is_empty()) {
        std::fs::write(out, csv).unwrap();
    }
    println!("{n} candidates, {real_guesses} from their real initial guess");
    for (v, (name, _, resolution)) in variants.iter().enumerate() {
        println!(
            "{:<26} resolution {:.3} m  {:7.1} ms/ICP  pass {:4}  flips vs as-is {:3}",
            name,
            resolution,
            1000.0 * time[v] / n.max(1) as f64,
            passes[v],
            flips[v]
        );
    }
}
