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

//! Scan-context lookup time vs map size, over N keyframes perturbed from a run's real `<run>/icp` scan descriptors.
//! usage: scan_context_bench <run dir> <full|pool> <memory GB> [N,N,...]
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::similarity_discovery::{
    auto_max_range, best_distance, make_descriptor, make_ring_key, order_closest,
    ring_key_distances, Config, Descriptor, RingKey,
};
use std::time::Instant;

/// Deterministic xorshift noise.
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
    /// Approximately standard normal (sum of 4 uniforms).
    fn normal(&mut self) -> f32 {
        ((0..4).map(|_| self.uniform()).sum::<f32>() - 2.0) * 3f32.sqrt()
    }
}

/// The corrected (rotation, translation) of each keyframe in the run's a.kf.
fn keyframe_poses(run: &std::path::Path) -> Vec<(mat3::Mat3, mat3::Vec3)> {
    let bytes = std::fs::read(run.join("a.kf")).expect("read a.kf");
    bytes
        .as_chunks::<{ 15 * 8 }>()
        .0
        .iter()
        .map(|record| {
            let f = |i: usize| f64::from_le_bytes(record[8 * i..8 * i + 8].try_into().unwrap());
            (
                mat3::mat_from_quat(&[f(14), f(11), f(12), f(13)]),
                [f(8), f(9), f(10)],
            )
        })
        .collect()
}

/// Up to `limit` descriptors of dumped single-ICP source scans, moved into their keyframe's body frame.
fn real_descriptors(run: &std::path::Path, limit: usize) -> (Vec<Descriptor>, Config) {
    let poses = keyframe_poses(run);
    let mut dumps: Vec<(usize, std::path::PathBuf)> = std::fs::read_dir(run.join("icp"))
        .expect("read icp dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter_map(|p| {
            let stem = p.file_stem()?.to_str()?.to_string();
            let mut parts = stem.split('_');
            let source = parts.next()?.parse().ok()?;
            (parts.count() == 1).then_some((source, p))
        })
        .collect();
    dumps.sort();
    dumps.dedup_by_key(|d| d.0);
    let mut config = Config::default();
    let mut descriptors = Vec::new();
    for (source, path) in dumps.into_iter().take(limit) {
        let Some(&(rotation, translation)) = poses.get(source) else {
            continue;
        };
        let bytes = std::fs::read(&path).unwrap();
        let ns = u32::from_le_bytes(bytes[0..4].try_into().unwrap()) as usize;
        let back = mat3::transpose(&rotation);
        let body: Vec<[f32; 3]> = bytes[8..8 + 12 * ns]
            .as_chunks::<12>()
            .0
            .iter()
            .map(|c| {
                let p = [0, 4, 8].map(|i| f32::from_le_bytes(c[i..i + 4].try_into().unwrap()) as f64);
                mat3::mat_vec(&back, &mat3::sub(&p, &translation)).map(|v| v as f32)
            })
            .collect();
        if config.max_range_m <= 0.0 || descriptors.is_empty() {
            config.max_range_m = auto_max_range(&body);
        }
        descriptors.push(make_descriptor(&body, &config));
    }
    (descriptors, config)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let run = std::path::Path::new(&args[1]);
    let full = args[2] == "full";
    let budget_bytes = args[3].parse::<f64>().unwrap() * (1u64 << 30) as f64;
    let sizes: Vec<usize> = args.get(4).map_or_else(
        || vec![1_000, 10_000, 100_000, 1_000_000, 5_000_000, 10_000_000, 20_000_000],
        |s| s.split(',').map(|n| n.parse::<f64>().unwrap() as usize).collect(),
    );
    let header = std::mem::size_of::<Vec<f32>>();
    let (pool, config) = real_descriptors(run, 2000);
    let pool_keys: Vec<RingKey> = pool.iter().map(make_ring_key).collect();
    // ring-key perturbation sd: 5% of the mean key value
    let mean_key = pool_keys.iter().flatten().sum::<f32>() / (pool_keys.len() * config.n_rings) as f32;
    let sd = 0.05 * mean_key;
    println!(
        "{} real descriptors ({} x {}, range {:.1} m), ring-key perturbation sd {sd:.3}",
        pool.len(),
        config.n_rings,
        config.n_sectors,
        config.max_range_m
    );
    let mut noise = Noise(0x9e3779b97f4a7c15);
    let perturbed = |key: &RingKey, noise: &mut Noise| -> RingKey {
        key.iter().map(|&v| (v + sd * noise.normal()).max(0.0)).collect()
    };
    // a descriptor's occupied cells moved by the same relative noise (empty cells stay empty)
    let perturbed_descriptor = |descriptor: &Descriptor, noise: &mut Noise| -> Descriptor {
        let mut copy = descriptor.clone();
        for v in copy.data.iter_mut().filter(|v| **v > 0.0) {
            *v = (*v * (1.0 + 0.05 * noise.normal())).max(1e-3);
        }
        copy
    };
    // bytes per keyframe: ring key (+ descriptor with `full`) plus its ranked entry
    let per_keyframe = (header + 4 * config.n_rings)
        + if full {
            header + 4 * config.n_rings * config.n_sectors + std::mem::size_of::<Descriptor>()
        } else {
            0
        }
        + std::mem::size_of::<(f32, usize)>();
    let max_n = (budget_bytes / per_keyframe as f64) as usize;
    println!("{} mode, {per_keyframe} B per keyframe: at most {max_n} keyframes in the budget", if full { "full" } else { "pool" });
    let mut keys: Vec<RingKey> = Vec::new();
    let mut descriptors: Vec<Descriptor> = Vec::new();
    let mut inserted = 0.0;
    println!("{:>10} {:>12} {:>12} {:>12} {:>8}", "N", "insert s", "lookup us", "rank us", "lookups");
    for n in sizes.iter().copied().filter(|&n| n <= max_n).chain(Some(max_n)) {
        if n < keys.len() {
            continue;
        }
        let started = Instant::now();
        while keys.len() < n {
            let real = keys.len() % pool.len();
            if full {
                let descriptor = perturbed_descriptor(&pool[real], &mut noise);
                keys.push(make_ring_key(&descriptor));
                descriptors.push(descriptor);
            } else {
                keys.push(perturbed(&pool_keys[real], &mut noise));
            }
        }
        inserted += started.elapsed().as_secs_f64();
        let lookups = (2_000_000_000 / (n.max(1) * 20)).clamp(15, 2001);
        let (mut times, mut rank_times) = (Vec::with_capacity(lookups), Vec::with_capacity(lookups));
        for l in 0..lookups {
            let which = (l * 7919) % pool.len();
            let query_key = perturbed(&pool_keys[which], &mut noise);
            let query = &pool[which];
            let started = Instant::now();
            // the same calls `search_by_scan_context` makes
            let mut ranked = ring_key_distances(&query_key, &keys);
            order_closest(&mut ranked, 0, config.candidate_top_k);
            let ranked_s = started.elapsed().as_secs_f64();
            let mut best = (2.0f32, 0i32);
            for &(_, idx) in ranked.iter().take(config.candidate_top_k) {
                let candidate = if full {
                    &descriptors[idx]
                } else {
                    &pool[idx % pool.len()]
                };
                let scored = best_distance(query, candidate);
                if scored.0 < best.0 {
                    best = scored;
                }
            }
            times.push(started.elapsed().as_secs_f64());
            rank_times.push(ranked_s);
            std::hint::black_box((best, ranked));
        }
        times.sort_by(f64::total_cmp);
        rank_times.sort_by(f64::total_cmp);
        println!(
            "{n:>10} {inserted:>12.2} {:>12.1} {:>12.1} {lookups:>8}",
            1e6 * times[times.len() / 2],
            1e6 * rank_times[rank_times.len() / 2]
        );
    }
}
