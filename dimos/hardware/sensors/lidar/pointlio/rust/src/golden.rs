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

// Golden output reader/writer: `frames.bin` + `trajectory.tum`. Layout: REPLAY.md "Golden output".

use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::Path;

pub const STATE_LEN: usize = 42;
pub const P_LEN: usize = 900;

/// `state[42]` block offsets, in dump order.
pub const STATE_BLOCKS: [(&str, usize, usize); 10] = [
    ("pos", 0, 3),
    ("rot", 3, 12),
    ("offset_R", 12, 21),
    ("offset_T", 21, 24),
    ("vel", 24, 27),
    ("omg", 27, 30),
    ("acc", 30, 33),
    ("gravity", 33, 36),
    ("bg", 36, 39),
    ("ba", 39, 42),
];

/// Per downsampled point: selection, its `Nearest_Points`, and the recomputed plane.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointDump {
    pub selected: bool,
    pub neighbours: Vec<[f32; 3]>,
    pub plane: [f32; 4],
}

/// One frame of intermediates.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub idx: u32,
    pub lidar_ts: f64,
    pub feats_down_body: Vec<[f32; 3]>,
    pub n_eff: u32,
    pub state: [f64; STATE_LEN],
    /// 30×30 row-major.
    pub p: Vec<f64>,
    pub points: Vec<PointDump>,
}

/// One `trajectory.tum` line: `ts x y z qx qy qz qw`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub ts: f64,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
}

impl Frame {
    pub fn write_to(&self, w: &mut impl Write) -> io::Result<()> {
        let mut b = Vec::new();
        b.extend_from_slice(&self.idx.to_le_bytes());
        b.extend_from_slice(&self.lidar_ts.to_le_bytes());
        b.extend_from_slice(&(self.feats_down_body.len() as u32).to_le_bytes());
        for p in &self.feats_down_body {
            xyz(&mut b, p);
        }
        b.extend_from_slice(&self.n_eff.to_le_bytes());
        assert_eq!(self.p.len(), P_LEN);
        for v in self.state.iter().chain(&self.p) {
            b.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(self.points.len(), self.feats_down_body.len());
        for pt in &self.points {
            b.push(u8::from(pt.selected));
            b.push(u8::try_from(pt.neighbours.len()).expect("n_nbr fits u8"));
            for n in &pt.neighbours {
                xyz(&mut b, n);
            }
            for v in pt.plane {
                b.extend_from_slice(&v.to_le_bytes());
            }
        }
        w.write_all(&b)
    }

    fn read_from(r: &mut impl Read) -> io::Result<Option<Self>> {
        let mut idx = [0u8; 4];
        match r.read(&mut idx)? {
            0 => return Ok(None),
            4 => {}
            n => r.read_exact(&mut idx[n..])?,
        }
        let idx = u32::from_le_bytes(idx);
        let lidar_ts = f64::from_le_bytes(arr(r)?);
        let n_down = u32::from_le_bytes(arr(r)?) as usize;
        let feats_down_body = (0..n_down)
            .map(|_| read_xyz(r))
            .collect::<io::Result<Vec<_>>>()?;
        let n_eff = u32::from_le_bytes(arr(r)?);
        let mut state = [0f64; STATE_LEN];
        for v in &mut state {
            *v = f64::from_le_bytes(arr(r)?);
        }
        let mut p = vec![0f64; P_LEN];
        for v in &mut p {
            *v = f64::from_le_bytes(arr(r)?);
        }
        let mut points = Vec::with_capacity(n_down);
        for _ in 0..n_down {
            let selected = arr::<1>(r)?[0] != 0;
            let n_nbr = arr::<1>(r)?[0] as usize;
            let neighbours = (0..n_nbr)
                .map(|_| read_xyz(r))
                .collect::<io::Result<Vec<_>>>()?;
            let mut plane = [0f32; 4];
            for v in &mut plane {
                *v = f32::from_le_bytes(arr(r)?);
            }
            points.push(PointDump {
                selected,
                neighbours,
                plane,
            });
        }
        Ok(Some(Frame {
            idx,
            lidar_ts,
            feats_down_body,
            n_eff,
            state,
            p,
            points,
        }))
    }
}

impl Pose {
    /// Same `%.9f` line as the C++ harness.
    pub fn write_to(&self, w: &mut impl Write) -> io::Result<()> {
        let [x, y, z] = self.pos;
        let [qx, qy, qz, qw] = self.quat;
        writeln!(
            w,
            "{:.9} {x:.9} {y:.9} {z:.9} {qx:.9} {qy:.9} {qz:.9} {qw:.9}",
            self.ts
        )
    }
}

pub fn read_frames(path: impl AsRef<Path>) -> io::Result<Vec<Frame>> {
    let mut r = BufReader::new(File::open(path)?);
    let mut frames = Vec::new();
    while let Some(f) = Frame::read_from(&mut r)? {
        frames.push(f);
    }
    Ok(frames)
}

pub fn read_tum(path: impl AsRef<Path>) -> io::Result<Vec<Pose>> {
    let mut poses = Vec::new();
    for (i, line) in BufReader::new(File::open(path)?).lines().enumerate() {
        let line = line?;
        let v: Vec<f64> = line
            .split_whitespace()
            .map(|s| s.parse().map_err(io::Error::other))
            .collect::<io::Result<_>>()?;
        let [ts, x, y, z, qx, qy, qz, qw] = v[..] else {
            return Err(io::Error::other(format!(
                "tum line {}: want 8 fields",
                i + 1
            )));
        };
        poses.push(Pose {
            ts,
            pos: [x, y, z],
            quat: [qx, qy, qz, qw],
        });
    }
    Ok(poses)
}

fn xyz(b: &mut Vec<u8>, p: &[f32; 3]) {
    for v in p {
        b.extend_from_slice(&v.to_le_bytes());
    }
}

fn read_xyz(r: &mut impl Read) -> io::Result<[f32; 3]> {
    let b: [u8; 12] = arr(r)?;
    let f = |i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
    Ok([f(0), f(4), f(8)])
}

fn arr<const N: usize>(r: &mut impl Read) -> io::Result<[u8; N]> {
    let mut buf = [0u8; N];
    r.read_exact(&mut buf)?;
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../../../../data/pointlio_golden/mid360_athens_stairs_60s"
    );

    #[test]
    fn reads_real_golden() {
        let dir = Path::new(GOLDEN);
        if !dir.join("frames.bin").exists() {
            eprintln!("skip: {GOLDEN} missing (gitignored data)");
            return;
        }
        let frames = read_frames(dir.join("frames.bin")).unwrap();
        assert_eq!(frames.len(), 30);
        let tum = read_tum(dir.join("trajectory.tum")).unwrap();
        assert!(tum.len() >= 30);
        for (i, f) in frames.iter().enumerate() {
            assert_eq!(f.idx as usize, i);
            assert_eq!(f.points.len(), f.feats_down_body.len());
            let selected = f.points.iter().filter(|p| p.selected).count();
            assert_eq!(f.n_eff as usize, selected, "frame {i}");
            assert_eq!(f.lidar_ts, tum[i].ts, "frame {i}");
            if i > 0 {
                assert!(f.lidar_ts > frames[i - 1].lidar_ts, "frame {i}");
            }
        }
    }

    #[test]
    fn frame_round_trip() {
        let frame = Frame {
            idx: 7,
            lidar_ts: 1.5,
            feats_down_body: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            n_eff: 1,
            state: [0.25; STATE_LEN],
            p: (0..P_LEN).map(|i| i as f64).collect(),
            points: vec![
                PointDump {
                    selected: true,
                    neighbours: vec![[1.0, 1.0, 1.0]; 5],
                    plane: [0.0, 0.0, 1.0, -2.0],
                },
                PointDump::default(),
            ],
        };
        let mut buf = Vec::new();
        frame.write_to(&mut buf).unwrap();
        frame.write_to(&mut buf).unwrap();
        let mut cur = &buf[..];
        let mut back = Vec::new();
        while let Some(f) = Frame::read_from(&mut cur).unwrap() {
            back.push(f);
        }
        assert_eq!(back, vec![frame.clone(), frame]);
    }

    #[test]
    fn pose_line_matches_printf() {
        let pose = Pose {
            ts: 103.141736208,
            pos: [-0.005329451, 0.003468813, 0.006029618],
            quat: [-0.392622901, -0.020033283, -0.001188510, 0.919480567],
        };
        let mut buf = Vec::new();
        pose.write_to(&mut buf).unwrap();
        assert_eq!(
            String::from_utf8(buf).unwrap(),
            "103.141736208 -0.005329451 0.003468813 0.006029618 -0.392622901 -0.020033283 -0.001188510 0.919480567\n"
        );
    }
}
