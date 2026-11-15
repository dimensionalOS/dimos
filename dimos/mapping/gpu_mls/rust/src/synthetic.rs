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

//! A ray-cast synthetic room for parity tests and benchmarks: floor, walls,
//! pillars, tables, a stair and a ramp, scanned by a spinning lidar.

pub type Point = (f32, f32, f32);

/// An incline rising `rise` along +x over its x extent.
struct Ramp {
    min: [f32; 2],
    max: [f32; 2],
    base_z: f32,
    rise: f32,
}

pub struct Scene {
    boxes: Vec<([f32; 3], [f32; 3])>,
    ramps: Vec<Ramp>,
    pub size: f32,
}

/// Deterministic pseudo-random stream, so scans are reproducible.
pub struct Lcg(pub u64);

impl Lcg {
    /// Uniform in [0, 1).
    pub fn next(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
}

impl Scene {
    /// A square room `size` meters on a side with `height` meter walls.
    pub fn room(size: f32, height: f32) -> Self {
        let wall = 0.2;
        let mut boxes = vec![
            ([-wall, -wall, -0.2], [size + wall, size + wall, 0.0]),
            ([-wall, -wall, 0.0], [0.0, size + wall, height]),
            ([size, -wall, 0.0], [size + wall, size + wall, height]),
            ([0.0, -wall, 0.0], [size, 0.0, height]),
            ([0.0, size, 0.0], [size, size + wall, height]),
        ];
        let step = size / 6.0;
        for i in 1..6 {
            for j in 1..6 {
                let (cx, cy) = (i as f32 * step, j as f32 * step);
                match (i + j) % 3 {
                    0 => boxes.push(([cx - 0.2, cy - 0.2, 0.0], [cx + 0.2, cy + 0.2, height])),
                    1 => boxes.push(([cx - 0.6, cy - 0.4, 0.7], [cx + 0.6, cy + 0.4, 0.75])),
                    _ => boxes.push(([cx - 0.5, cy - 0.5, 0.0], [cx + 0.5, cy + 0.5, 0.45])),
                }
            }
        }
        // A five-step stair along +y.
        let (sx, sy) = (0.15 * size, 0.45 * size);
        for k in 0..5 {
            let y0 = sy + 0.3 * k as f32;
            boxes.push(([sx, y0, 0.0], [sx + 1.5, y0 + 0.3, 0.15 * (k + 1) as f32]));
        }
        let ramps = vec![Ramp {
            min: [0.55 * size, 0.1 * size],
            max: [0.55 * size + 4.0, 0.1 * size + 1.5],
            base_z: 0.0,
            rise: 0.5,
        }];
        Self { boxes, ramps, size }
    }

    /// Distance along unit `dir` to the first surface, if any within `max_range`.
    pub fn cast(&self, origin: [f32; 3], dir: [f32; 3], max_range: f32) -> Option<f32> {
        let mut best = max_range;
        for (lo, hi) in &self.boxes {
            let (mut t0, mut t1) = (0.0f32, best);
            let mut hit = true;
            for a in 0..3 {
                if dir[a].abs() < 1e-9 {
                    if origin[a] < lo[a] || origin[a] > hi[a] {
                        hit = false;
                        break;
                    }
                    continue;
                }
                let inv = 1.0 / dir[a];
                let (mut ta, mut tb) = ((lo[a] - origin[a]) * inv, (hi[a] - origin[a]) * inv);
                if ta > tb {
                    std::mem::swap(&mut ta, &mut tb);
                }
                t0 = t0.max(ta);
                t1 = t1.min(tb);
                if t0 > t1 {
                    hit = false;
                    break;
                }
            }
            if hit && t0 > 0.0 && t0 < best {
                best = t0;
            }
        }
        for r in &self.ramps {
            let slope = r.rise / (r.max[0] - r.min[0]);
            // z = base + slope (x - min_x), as n . p = c with n = (-slope, 0, 1).
            let denom = dir[2] - slope * dir[0];
            if denom.abs() < 1e-9 {
                continue;
            }
            let c = r.base_z - slope * r.min[0];
            let t = (c - (origin[2] - slope * origin[0])) / denom;
            if t <= 0.0 || t >= best {
                continue;
            }
            let (x, y) = (origin[0] + t * dir[0], origin[1] + t * dir[1]);
            if x >= r.min[0] && x <= r.max[0] && y >= r.min[1] && y <= r.max[1] {
                best = t;
            }
        }
        (best < max_range).then_some(best)
    }

    /// One spinning-lidar frame of about `count` returns from `origin`, with
    /// range noise of `noise` meters.
    pub fn scan(
        &self,
        origin: Point,
        count: usize,
        rng: &mut Lcg,
        noise: f32,
        max_range: f32,
    ) -> Vec<Point> {
        const ROWS: usize = 32;
        let (lo, hi) = (-35f32.to_radians(), 25f32.to_radians());
        let cols = count.div_ceil(ROWS);
        let spin = rng.next() * std::f32::consts::TAU;
        let o = [origin.0, origin.1, origin.2];
        let mut out = Vec::with_capacity(count);
        for r in 0..ROWS {
            let el = lo + (hi - lo) * (r as f32 + rng.next()) / ROWS as f32;
            for c in 0..cols {
                let az = spin + std::f32::consts::TAU * (c as f32 + rng.next()) / cols as f32;
                let dir = [el.cos() * az.cos(), el.cos() * az.sin(), el.sin()];
                if let Some(t) = self.cast(o, dir, max_range) {
                    let t = t + noise * (rng.next() * 2.0 - 1.0);
                    out.push((o[0] + t * dir[0], o[1] + t * dir[1], o[2] + t * dir[2]));
                }
            }
        }
        out
    }

    /// A loop through the room at sensor height, one origin per frame.
    pub fn trajectory(&self, frames: usize, height: f32) -> Vec<Point> {
        let c = self.size / 2.0;
        let radius = 0.3 * self.size;
        (0..frames)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / frames.max(1) as f32;
                (c + radius * a.cos(), c + 0.8 * radius * a.sin(), height)
            })
            .collect()
    }
}
