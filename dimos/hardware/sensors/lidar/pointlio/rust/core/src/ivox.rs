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

// ivox3d.h + ivox3d_node.hpp (linear node): LRU voxel hash map with k-NN over a fixed stencil.
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use crate::common::PointXYZI;
use crate::sort::nth_element;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NearbyType {
    Center,
    Nearby6,
    Nearby18,
    Nearby26,
}

#[derive(Clone, Copy, Debug)]
pub struct IVoxOptions {
    pub resolution: f32,
    pub nearby_type: NearbyType,
    pub capacity: usize,
}

impl Default for IVoxOptions {
    fn default() -> Self {
        Self {
            resolution: 0.2,
            nearby_type: NearbyType::Nearby6,
            capacity: 1_000_000,
        }
    }
}

pub type Key = [i32; 3];

const NIL: usize = usize::MAX;

// FxHash-style multiply-rotate over the three i32 lanes; SipHash was measurable in the profile.
#[derive(Default)]
struct KeyHasher(u64);

impl Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for lane in bytes.chunks_exact(4) {
            let v = u32::from_ne_bytes(lane.try_into().unwrap()) as u64;
            self.0 = (self.0.rotate_left(5) ^ v).wrapping_mul(0x517c_c1b7_2722_0a95);
        }
    }

    fn write_usize(&mut self, _len: usize) {}
}

struct Voxel {
    key: Key,
    points: Vec<PointXYZI>,
    prev: usize,
    next: usize,
}

pub struct IVox {
    inv_resolution: f32,
    capacity: usize,
    nearby: Vec<Key>,
    map: HashMap<Key, usize, BuildHasherDefault<KeyHasher>>,
    voxels: Vec<Voxel>,
    free: Vec<usize>,
    head: usize,
    tail: usize,
    cands: Vec<Cand>,
}

// 16 bytes: the partition loops in `nth_element` are memory-bound.
#[derive(Clone, Copy)]
struct Cand {
    dist: f64,
    slot: u32,
    idx: u32,
}

impl IVox {
    pub fn new(opts: IVoxOptions) -> Self {
        Self {
            inv_resolution: (1.0f64 / opts.resolution as f64) as f32,
            capacity: opts.capacity,
            nearby: nearby_grids(opts.nearby_type),
            map: HashMap::default(),
            voxels: Vec::new(),
            free: Vec::new(),
            head: NIL,
            tail: NIL,
            cands: Vec::new(),
        }
    }

    pub fn pos2grid(&self, x: f32, y: f32, z: f32) -> Key {
        let g = |v: f32| (v * self.inv_resolution).floor() as i32;
        [g(x), g(y), g(z)]
    }

    pub fn num_valid_grids(&self) -> usize {
        self.map.len()
    }

    pub fn add_points(&mut self, points: &[PointXYZI]) {
        for &p in points {
            let key = self.pos2grid(p.x, p.y, p.z);
            match self.map.get(&key).copied() {
                None => {
                    let slot = self.alloc(key, p);
                    self.push_front(slot);
                    self.map.insert(key, slot);
                    if self.map.len() >= self.capacity {
                        let back = self.tail;
                        self.unlink(back);
                        self.map.remove(&self.voxels[back].key);
                        self.voxels[back].points = Vec::new();
                        self.free.push(back);
                    }
                }
                Some(slot) => {
                    self.voxels[slot].points.push(p);
                    self.unlink(slot);
                    self.push_front(slot);
                }
            }
        }
    }

    /// Up to `max_num` nearest map points within `max_range`, in the C++ `nth_element` order
    /// (first is the closest), written into `out`; false and `out` untouched when none
    /// (GetClosestPoint leaves its output stale then).
    pub fn closest_points(
        &mut self,
        pt: &PointXYZI,
        max_num: usize,
        max_range: f64,
        out: &mut Vec<PointXYZI>,
    ) -> bool {
        let key = self.pos2grid(pt.x, pt.y, pt.z);
        let mut cands = std::mem::take(&mut self.cands);
        cands.clear();
        for d in &self.nearby {
            let dkey = [key[0] + d[0], key[1] + d[1], key[2] + d[2]];
            if let Some(&slot) = self.map.get(&dkey) {
                self.knn_by_condition(slot, &mut cands, pt, max_num, max_range);
            }
        }
        let found = !cands.is_empty();
        if found {
            if cands.len() > max_num {
                nth_element(&mut cands, max_num - 1, &less);
                cands.truncate(max_num);
            }
            nth_element(&mut cands, 0, &less);
            out.clear();
            out.extend(
                cands
                    .iter()
                    .map(|c| self.voxels[c.slot as usize].points[c.idx as usize]),
            );
        }
        self.cands = cands;
        found
    }

    fn knn_by_condition(
        &self,
        slot: usize,
        cands: &mut Vec<Cand>,
        pt: &PointXYZI,
        k: usize,
        max_range: f64,
    ) {
        let old = cands.len();
        for (idx, q) in self.voxels[slot].points.iter().enumerate() {
            let d = distance2(q, pt);
            if d < max_range * max_range {
                cands.push(Cand {
                    dist: d,
                    slot: slot as u32,
                    idx: idx as u32,
                });
            }
        }
        if old + k < cands.len() {
            nth_element(&mut cands[old..], k - 1, &less);
            cands.truncate(old + k);
        }
    }

    fn alloc(&mut self, key: Key, p: PointXYZI) -> usize {
        let v = Voxel {
            key,
            points: vec![p],
            prev: NIL,
            next: NIL,
        };
        match self.free.pop() {
            Some(slot) => {
                self.voxels[slot] = v;
                slot
            }
            None => {
                self.voxels.push(v);
                self.voxels.len() - 1
            }
        }
    }

    fn push_front(&mut self, slot: usize) {
        self.voxels[slot].prev = NIL;
        self.voxels[slot].next = self.head;
        if self.head != NIL {
            self.voxels[self.head].prev = slot;
        } else {
            self.tail = slot;
        }
        self.head = slot;
    }

    fn unlink(&mut self, slot: usize) {
        let (prev, next) = (self.voxels[slot].prev, self.voxels[slot].next);
        if prev != NIL {
            self.voxels[prev].next = next;
        } else {
            self.head = next;
        }
        if next != NIL {
            self.voxels[next].prev = prev;
        } else {
            self.tail = prev;
        }
    }
}

// Squared distance in f32 like `distance2` (Vector3f::squaredNorm sums as a0 + (a1 + a2)).
fn distance2(a: &PointXYZI, b: &PointXYZI) -> f64 {
    let (dx, dy, dz) = (a.x - b.x, a.y - b.y, a.z - b.z);
    (dx * dx + (dy * dy + dz * dz)) as f64
}

// `DistPoint::operator<` (by squared distance).
fn less(a: &Cand, b: &Cand) -> bool {
    a.dist < b.dist
}

fn nearby_grids(t: NearbyType) -> Vec<Key> {
    #[rustfmt::skip]
    const N26: [Key; 27] = [
        [0, 0, 0], [-1, 0, 0], [1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1], [0, 0, 1],
        [1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0], [1, 0, 1], [-1, 0, 1], [1, 0, -1],
        [-1, 0, -1], [0, 1, 1], [0, -1, 1], [0, 1, -1], [0, -1, -1],
        [1, 1, 1], [-1, 1, 1], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [-1, 1, -1], [1, -1, -1], [-1, -1, -1],
    ];
    let n = match t {
        NearbyType::Center => 1,
        NearbyType::Nearby6 => 7,
        NearbyType::Nearby18 => 19,
        NearbyType::Nearby26 => 27,
    };
    N26[..n].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f32, y: f32, z: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 0.0,
            curvature: 0.0,
        }
    }

    #[test]
    fn grid_neighbours_and_lru() {
        let mut ivox = IVox::new(IVoxOptions {
            resolution: 1.0,
            nearby_type: NearbyType::Nearby6,
            capacity: 1000,
        });
        assert_eq!(ivox.pos2grid(-0.5, 0.0, 1.999), [-1, 0, 1]);
        let pts = [
            pt(0.5, 0.5, 0.5), // centre voxel
            pt(1.5, 0.5, 0.5), // +x face
            pt(0.5, 1.9, 0.5), // +y face
            pt(1.5, 1.5, 0.5), // diagonal: not in nearby6
            pt(0.6, 0.5, 0.5), // centre voxel, closest
            pt(4.0, 4.0, 4.0), // out of range
        ];
        ivox.add_points(&pts);
        assert_eq!(ivox.num_valid_grids(), 5);
        let q = pt(0.7, 0.5, 0.5);
        let near = |ivox: &mut IVox, q: &PointXYZI, k: usize, r: f64| {
            let mut out = Vec::new();
            ivox.closest_points(q, k, r, &mut out);
            out
        };
        let got5 = near(&mut ivox, &q, 5, 5.0);
        assert_eq!(got5[0], pts[4]);
        let mut got: Vec<_> = got5.iter().map(|p| (p.x, p.y, p.z)).collect();
        got.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(
            got,
            vec![
                (0.5, 0.5, 0.5),
                (0.5, 1.9, 0.5),
                (0.6, 0.5, 0.5),
                (1.5, 0.5, 0.5)
            ]
        );
        assert_eq!(near(&mut ivox, &q, 2, 5.0).len(), 2);
        assert!(near(&mut ivox, &q, 5, 0.05).is_empty());
        assert!(near(&mut ivox, &pt(9.0, 9.0, 9.0), 5, 5.0).is_empty());

        // Capacity: inserting the 3rd voxel evicts the least recently touched one.
        let mut small = IVox::new(IVoxOptions {
            resolution: 1.0,
            nearby_type: NearbyType::Center,
            capacity: 3,
        });
        small.add_points(&[
            pt(0.5, 0.5, 0.5),
            pt(1.5, 0.5, 0.5),
            pt(0.5, 0.5, 0.5),
            pt(2.5, 0.5, 0.5),
        ]);
        assert_eq!(small.num_valid_grids(), 2);
        assert!(near(&mut small, &pt(1.5, 0.5, 0.5), 5, 5.0).is_empty());
        assert_eq!(near(&mut small, &pt(0.5, 0.5, 0.5), 5, 5.0).len(), 2);
    }
}
