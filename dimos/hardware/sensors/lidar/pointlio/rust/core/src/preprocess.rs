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

// preprocess.cpp, Livox CustomMsg path (`avia_handler`) only.
use crate::common::PointXYZI;

/// The Livox CustomPoint fields the handler reads (`offset_time` in ns).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LivoxPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflectivity: u16,
    pub tag: u8,
    pub line: u16,
    pub offset_time: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct Preprocess {
    pub blind: f64,
    pub point_filter_num: u32,
    pub n_scans: u16,
    pub det_range: f64,
}

impl Default for Preprocess {
    fn default() -> Self {
        Self {
            blind: 0.01,
            point_filter_num: 1,
            n_scans: 6,
            det_range: 1000.0,
        }
    }
}

impl Preprocess {
    /// Filters a frame into the surf cloud, same order and conditions as the C++ (point 0 is
    /// never emitted; the dedup compares against slot i-1 of the sparse full cloud).
    pub fn process(&self, points: &[LivoxPoint]) -> Vec<PointXYZI> {
        let mut full = vec![PointXYZI::default(); points.len()];
        let mut surf = Vec::with_capacity(points.len());
        let mut valid_num = 0u32;
        for i in 1..points.len() {
            let p = &points[i];
            if p.line >= self.n_scans || (p.tag & 0x30 != 0x10 && p.tag & 0x30 != 0x00) {
                continue;
            }
            valid_num += 1;
            if !valid_num.is_multiple_of(self.point_filter_num) {
                continue;
            }
            let q = PointXYZI {
                x: p.x,
                y: p.y,
                z: p.z,
                intensity: p.reflectivity as f32,
                curvature: p.offset_time as f32 / 1_000_000.0,
            };
            full[i] = q;
            let dist = (q.x * q.x + q.y * q.y + q.z * q.z) as f64;
            if dist < self.blind * self.blind || dist > self.det_range * self.det_range {
                continue;
            }
            let prev = full[i - 1];
            if (q.x - prev.x).abs() as f64 > 1e-7
                || (q.y - prev.y).abs() as f64 > 1e-7
                || (q.z - prev.z).abs() as f64 > 1e-7
            {
                surf.push(q);
            }
        }
        surf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lp(x: f32, tag: u8, offset_time: u64) -> LivoxPoint {
        LivoxPoint {
            x,
            y: 0.0,
            z: 0.0,
            reflectivity: 7,
            tag,
            line: 0,
            offset_time,
        }
    }

    #[test]
    fn filters_like_avia_handler() {
        let pts = [
            lp(9.0, 0, 0),       // index 0: always skipped
            lp(1.0, 0, 1000),    // valid 1
            lp(2.0, 0x20, 0),    // bad tag, not counted
            lp(3.0, 0x10, 2000), // valid 2 -> filtered out (every 2nd)
            lp(0.001, 0, 3000),  // valid 3: blind
            lp(5.0, 0, 4000),    // valid 4 -> filtered out
            lp(5.0, 0, 5000),    // valid 5: prev slot (index 5) was zero, so emitted
            lp(5.0, 0, 6000),    // valid 6 -> filtered out
            lp(5.0, 0, 7000),    // valid 7: prev slot zero again -> emitted
        ];
        let out = Preprocess {
            blind: 0.5,
            point_filter_num: 2,
            ..Default::default()
        }
        .process(&pts);
        let xs: Vec<f32> = out.iter().map(|p| p.x).collect();
        assert_eq!(xs, vec![3.0, 5.0, 5.0]);
        assert_eq!(out[0].curvature, 0.002);
        assert_eq!(out[0].intensity, 7.0);

        let out = Preprocess {
            blind: 0.5,
            ..Default::default()
        }
        .process(&pts);
        let xs: Vec<f32> = out.iter().map(|p| p.x).collect();
        assert_eq!(xs, vec![1.0, 3.0, 5.0]); // blind point 4 still fills its slot; 6..8 duplicate 5
        let mut bad_line = pts;
        bad_line[1].line = 6;
        assert_eq!(
            Preprocess {
                blind: 0.5,
                ..Default::default()
            }
            .process(&bad_line)
            .len(),
            2
        );
    }
}
