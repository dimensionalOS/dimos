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

// L1/L2 parity against the C++ harness golden (REPLAY.md). Skips when the gitignored data is
// missing. `--release` replays the whole file; debug builds stop at the golden frames.

use std::fs;
use std::path::Path;

use pointlio_core::{Config, FrameDump, LivoxPoint, PointLio};

const DATA: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../../../../../data");
const GRAVITY_MS2: f64 = 9.80665;

struct Rd<'a>(&'a [u8]);

impl Rd<'_> {
    fn take(&mut self, n: usize) -> &[u8] {
        let (a, b) = self.0.split_at(n);
        self.0 = b;
        a
    }
    fn u8(&mut self) -> u8 {
        self.take(1)[0]
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take(8).try_into().unwrap())
    }
    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take(8).try_into().unwrap())
    }
    fn xyz(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }
}

fn read_frames(bytes: &[u8]) -> Vec<FrameDump> {
    let mut r = Rd(bytes);
    let mut out = Vec::new();
    while !r.0.is_empty() {
        let _idx = r.u32();
        let lidar_ts = r.f64();
        let n = r.u32() as usize;
        let feats_down_body = (0..n).map(|_| r.xyz()).collect();
        let n_eff = r.u32();
        let state: [f64; 42] = std::array::from_fn(|_| r.f64());
        let p = (0..900).map(|_| r.f64()).collect();
        let points = (0..n)
            .map(|_| {
                let selected = r.u8() != 0;
                let k = r.u8() as usize;
                let neighbours = (0..k).map(|_| r.xyz()).collect();
                let plane = std::array::from_fn(|_| r.f32());
                pointlio_core::PointDump {
                    selected,
                    neighbours,
                    plane,
                }
            })
            .collect();
        out.push(FrameDump {
            lidar_ts,
            feats_down_body,
            n_eff,
            state,
            p,
            points,
        });
    }
    out
}

fn ulp_diff(a: f32, b: f32) -> u32 {
    if a == b {
        return 0;
    }
    (a.to_bits() as i64 - b.to_bits() as i64)
        .unsigned_abs()
        .min(u32::MAX as u64) as u32
}

fn max_rel(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs() / x.abs().max(1.0))
        .fold(0.0, f64::max)
}

#[test]
fn first_frames_match_cpp_golden() {
    let data = Path::new(DATA);
    let replay = data.join("pointlio_replay/mid360_athens_stairs_60s.plio");
    let golden = data.join("pointlio_golden/mid360_athens_stairs_60s");
    let (Ok(plio), Ok(frames_bin), Ok(cfg_json), Ok(tum)) = (
        fs::read(&replay),
        fs::read(golden.join("frames.bin")),
        fs::read_to_string(golden.join("config.json")),
        fs::read_to_string(golden.join("trajectory.tum")),
    ) else {
        eprintln!("skip: golden data missing under {}", data.display());
        return;
    };
    let cfg: Config = serde_json::from_str(&cfg_json).unwrap();
    let want = read_frames(&frames_bin);
    let tum: Vec<Vec<f64>> = tum
        .lines()
        .map(|l| l.split(' ').map(|v| v.parse().unwrap()).collect())
        .collect();
    // Debug builds run ~2 s/frame: stop at the golden frames unless told otherwise.
    let max_frames: usize = std::env::var("POINTLIO_GOLDEN_FRAMES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) {
            want.len()
        } else {
            usize::MAX
        });

    let mut lio = PointLio::new(&cfg);
    let mut r = Rd(&plio);
    assert_eq!(r.take(4), b"PLIO");
    assert_eq!(r.u32(), 1);
    r.f64();
    let mut processed = 0usize;
    let mut pts = Vec::new();
    while !r.0.is_empty() && processed < max_frames {
        match r.u8() {
            0 => {
                let ts = r.u64() as f64 / 1e9;
                let g = [r.f64(), r.f64(), r.f64()];
                let a = [r.f64(), r.f64(), r.f64()].map(|v| v / GRAVITY_MS2);
                lio.feed_imu(ts, g, a);
            }
            1 => {
                let start_ns = r.u64();
                let n = r.u32();
                pts.clear();
                for _ in 0..n {
                    let [x, y, z] = r.xyz();
                    let refl = (r.f32() * 255.0).round() as u16;
                    let offset_time = r.u32() as u64;
                    let tag = r.u8();
                    pts.push(LivoxPoint {
                        x,
                        y,
                        z,
                        reflectivity: refl,
                        tag,
                        line: 0,
                        offset_time,
                    });
                }
                lio.feed_lidar(start_ns, &pts);
                loop {
                    let before = lio.lidar_buffer_len();
                    if lio.process() {
                        check_frame(processed, &lio, &want, &tum);
                        processed += 1;
                    }
                    if lio.lidar_buffer_len() == before {
                        break;
                    }
                }
            }
            k => panic!("bad record kind {k}"),
        }
    }
    assert!(
        processed >= want.len().min(max_frames),
        "processed {processed}"
    );
    eprintln!(
        "processed {processed} frames ({} golden, {} tum)",
        want.len(),
        tum.len()
    );
}

fn check_frame(i: usize, lio: &PointLio, want: &[FrameDump], tum: &[Vec<f64>]) {
    let odom = lio.odometry();
    if let Some(t) = tum.get(i) {
        assert_eq!(
            format!("{:.9}", odom.ts),
            format!("{:.9}", t[0]),
            "frame {i} ts"
        );
        let got = [odom.pos, odom.quat[..3].try_into().unwrap()].concat();
        let got = [got, vec![odom.quat[3]]].concat();
        let err = got
            .iter()
            .zip(&t[1..])
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        // L2 on the golden frames; afterwards ULP-level selection flips make the two paths
        // diverge (chaos), so only a gross bound applies. L3 (noise-band APE) is the replay CLI's job.
        let traj_tol = if i < want.len() { 1e-6 } else { 0.5 };
        if i.is_multiple_of(50) || err >= traj_tol {
            eprintln!("frame {i}: trajectory abs err {err:.2e}");
        }
        assert!(
            err < traj_tol,
            "frame {i} trajectory off by {err}: {got:?} vs {t:?}"
        );
    }
    let Some(w) = want.get(i) else {
        return;
    };
    let g = lio.frame_dump();
    assert_eq!(g.lidar_ts, w.lidar_ts, "frame {i} ts");
    assert_eq!(
        g.feats_down_body.len(),
        w.feats_down_body.len(),
        "frame {i} n_down"
    );
    let bad: Vec<_> = g
        .feats_down_body
        .iter()
        .zip(&w.feats_down_body)
        .enumerate()
        .filter(|(_, (a, b))| a != b)
        .take(5)
        .collect();
    assert!(bad.is_empty(), "frame {i} feats_down_body differs: {bad:?}");
    assert_eq!(g.n_eff, w.n_eff, "frame {i} n_eff");
    let tol: f64 = std::env::var("POINTLIO_GOLDEN_TOL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1e-9);
    let se = max_rel(&g.state, &w.state);
    let pe = max_rel(&g.p, &w.p);
    let blocks = [
        ("pos", 0, 3),
        ("rot", 3, 12),
        ("vel", 24, 27),
        ("omg", 27, 30),
        ("acc", 30, 33),
        ("grav", 33, 36),
        ("bg", 36, 39),
        ("ba", 39, 42),
    ];
    let per: Vec<String> = blocks
        .iter()
        .map(|(n, a, b)| format!("{n}={:.1e}", max_rel(&g.state[*a..*b], &w.state[*a..*b])))
        .collect();
    if i + 1 == want.len() {
        eprintln!(
            "frame {i}: state rel err {se:.2e} P {pe:.2e} [{}]",
            per.join(" ")
        );
    }
    for (j, (a, b)) in g.points.iter().zip(&w.points).enumerate() {
        assert_eq!(a.selected, b.selected, "frame {i} point {j} selected");
        assert_eq!(a.neighbours, b.neighbours, "frame {i} point {j} neighbours");
        let ulp = a
            .plane
            .iter()
            .zip(&b.plane)
            .map(|(x, y)| ulp_diff(*x, *y))
            .max()
            .unwrap();
        assert!(
            ulp <= 4,
            "frame {i} point {j} plane {:?} vs {:?}",
            a.plane,
            b.plane
        );
    }
    assert!(
        se < tol,
        "frame {i} state rel err {se}\n got {:?}\nwant {:?}",
        g.state,
        w.state
    );
    assert!(pe < tol, "frame {i} P rel err {pe}");
}

/// Every selected golden point is a (neighbours, plane) pair: exact-bits check of `esti_plane`.
#[test]
fn esti_plane_matches_golden_bits() {
    let golden = Path::new(DATA).join("pointlio_golden/mid360_athens_stairs_60s");
    let (Ok(frames_bin), Ok(cfg_json)) = (
        fs::read(golden.join("frames.bin")),
        fs::read_to_string(golden.join("config.json")),
    ) else {
        eprintln!("skip: golden data missing");
        return;
    };
    let cfg: Config = serde_json::from_str(&cfg_json).unwrap();
    let want = read_frames(&frames_bin);
    let (mut total, mut exact, mut ulp4) = (0u32, 0u32, 0u32);
    let mut worst = (0u32, [0f32; 4], [0f32; 4], Vec::new());
    for f in &want {
        for p in &f.points {
            if !p.selected || p.neighbours.len() < 5 {
                continue;
            }
            let pts: Vec<_> = p
                .neighbours
                .iter()
                .map(|n| pointlio_core::PointXYZI {
                    x: n[0],
                    y: n[1],
                    z: n[2],
                    intensity: 0.0,
                    curvature: 0.0,
                })
                .collect();
            let got =
                pointlio_core::common::esti_plane(&pts, cfg.plane_thr as f32).unwrap_or([0.0; 4]);
            let ulp = got
                .iter()
                .zip(&p.plane)
                .map(|(a, b)| ulp_diff(*a, *b))
                .max()
                .unwrap();
            total += 1;
            exact += (ulp == 0) as u32;
            ulp4 += (ulp <= 4) as u32;
            if ulp > worst.0 {
                worst = (ulp, got, p.plane, p.neighbours.clone());
            }
        }
    }
    eprintln!(
        "esti_plane: {total} points, {exact} exact, {ulp4} within 4 ulp; worst {:?}",
        worst
    );
    assert_eq!(exact, total, "esti_plane bit mismatches: {}", total - exact);
}
