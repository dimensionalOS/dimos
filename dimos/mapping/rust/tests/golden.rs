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

//
// Golden tests: replay the recorded go2_short.db lidar frames through the
// Rust ports and compare against the Python implementations' outputs.
//
// Data is generated (not committed) by:
//     uv run python scripts/benchmarks/dump_golden.py
// Tests skip with a notice when the data is absent.
//
// Contract:
//   - voxel keys: EXACT set equality (integer math must not drift)
//   - costmap: same shape/origin; cells within +/-1 and <0.1% mismatches
//     (float pipeline; gaussian/sobel rounding differs in the last bits)

use std::fs;
use std::path::PathBuf;

use dimos_mappers::occupancy::{height_cost_occupancy, HeightCostConfig};
use dimos_mappers::voxel_grid::VoxelGrid;

const CHECKPOINTS: [usize; 5] = [5, 50, 150, 300, 460];
const VOXEL_SIZE: f32 = 0.05;

fn golden_dir() -> Option<PathBuf> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden_data");
    if dir.join("frames.bin").exists() {
        Some(dir)
    } else {
        eprintln!(
            "SKIP: no golden data at {dir:?}; generate with \
             `uv run python scripts/benchmarks/dump_golden.py`"
        );
        None
    }
}

fn read_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

/// frames.bin: repeated [u32 n][n * 3 f32 xyz].
fn read_frames(path: PathBuf) -> Vec<Vec<(f32, f32, f32)>> {
    let buf = fs::read(path).unwrap();
    let mut frames = Vec::new();
    let mut off = 0;
    while off < buf.len() {
        let n = read_u32(&buf, off) as usize;
        off += 4;
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let base = off + i * 12;
            pts.push((
                f32::from_le_bytes(buf[base..base + 4].try_into().unwrap()),
                f32::from_le_bytes(buf[base + 4..base + 8].try_into().unwrap()),
                f32::from_le_bytes(buf[base + 8..base + 12].try_into().unwrap()),
            ));
        }
        off += n * 12;
        frames.push(pts);
    }
    frames
}

/// voxels_N.bin: [u32 m][m * 3 i32], lexicographically sorted by Python.
fn read_keys(path: PathBuf) -> Vec<[i32; 3]> {
    let buf = fs::read(path).unwrap();
    let m = read_u32(&buf, 0) as usize;
    (0..m)
        .map(|i| {
            let base = 4 + i * 12;
            [
                i32::from_le_bytes(buf[base..base + 4].try_into().unwrap()),
                i32::from_le_bytes(buf[base + 4..base + 8].try_into().unwrap()),
                i32::from_le_bytes(buf[base + 8..base + 12].try_into().unwrap()),
            ]
        })
        .collect()
}

/// map_N.bin: [u32 m][m * 3 f32] voxel centers emitted by Python.
fn read_centers(path: PathBuf) -> Vec<(f64, f64, f64)> {
    let buf = fs::read(path).unwrap();
    let m = read_u32(&buf, 0) as usize;
    (0..m)
        .map(|i| {
            let base = 4 + i * 12;
            (
                f32::from_le_bytes(buf[base..base + 4].try_into().unwrap()) as f64,
                f32::from_le_bytes(buf[base + 4..base + 8].try_into().unwrap()) as f64,
                f32::from_le_bytes(buf[base + 8..base + 12].try_into().unwrap()) as f64,
            )
        })
        .collect()
}

#[test]
fn voxel_keys_match_python_exactly() {
    let Some(dir) = golden_dir() else { return };
    let frames = read_frames(dir.join("frames.bin"));

    let mut grid = VoxelGrid::default();
    for (i, frame) in frames.iter().enumerate() {
        grid.add_frame(frame, VOXEL_SIZE, true);
        let n = i + 1;
        if CHECKPOINTS.contains(&n) {
            let expected = read_keys(dir.join(format!("voxels_{n}.bin")));
            let got = grid.sorted_keys();
            assert_eq!(
                got.len(),
                expected.len(),
                "frame {n}: voxel count mismatch (rust {} vs python {})",
                got.len(),
                expected.len()
            );
            let mismatches = got.iter().zip(&expected).filter(|(a, b)| a != b).count();
            assert_eq!(mismatches, 0, "frame {n}: {mismatches} differing keys");
        }
    }
}

#[test]
fn costmap_matches_python_within_tolerance() {
    let Some(dir) = golden_dir() else { return };
    let cfg = HeightCostConfig {
        resolution: 0.05,
        can_pass_under: 0.6,
        can_climb: 0.15,
        ignore_noise: 0.05,
        smoothing: 1.0,
    };

    for n in CHECKPOINTS {
        let centers = read_centers(dir.join(format!("map_{n}.bin")));
        let grid = height_cost_occupancy(&centers, &cfg);

        let meta: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(dir.join(format!("cost_{n}.json"))).unwrap())
                .unwrap();
        let expected = fs::read(dir.join(format!("cost_{n}.bin"))).unwrap();

        assert_eq!(
            grid.width,
            meta["width"].as_u64().unwrap() as usize,
            "frame {n}: width"
        );
        assert_eq!(
            grid.height,
            meta["height"].as_u64().unwrap() as usize,
            "frame {n}: height"
        );
        assert!(
            (grid.origin_x - meta["origin_x"].as_f64().unwrap()).abs() < 1e-9,
            "frame {n}: origin_x"
        );
        assert!(
            (grid.origin_y - meta["origin_y"].as_f64().unwrap()).abs() < 1e-9,
            "frame {n}: origin_y"
        );

        let total = grid.data.len();
        assert_eq!(total, expected.len(), "frame {n}: cell count");
        let mut mismatches = 0usize;
        let mut max_diff = 0i32;
        for (i, &got) in grid.data.iter().enumerate() {
            let want = expected[i] as i8;
            if got != want {
                mismatches += 1;
                max_diff = max_diff.max((got as i32 - want as i32).abs());
            }
        }
        let rate = mismatches as f64 / total as f64;
        eprintln!(
            "frame {n}: {}x{} cells={total} mismatches={mismatches} ({:.4}%) max_diff={max_diff}",
            grid.width,
            grid.height,
            rate * 100.0
        );
        assert!(
            max_diff <= 1 && rate < 0.001,
            "frame {n}: costmap drift too large (mismatches={mismatches}, rate={rate:.5}, max_diff={max_diff})"
        );
    }
}
