// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::cmp::Ordering;

use glam::Vec3;

pub const LEAF_TRIANGLES: usize = 16;
pub const RAY_EPSILON: f32 = 1.0e-6;

#[derive(Clone, Copy, Debug, Default)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub min: Vec3,
    pub max: Vec3,
    pub centroid: Vec3,
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let min = a.min(b).min(c);
        let max = a.max(b).max(c);
        let centroid = (a + b + c) / 3.0;
        Self {
            a,
            b,
            c,
            min,
            max,
            centroid,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct BvhNode {
    min: Vec3,
    max: Vec3,
    left: Option<usize>,
    right: Option<usize>,
    start: usize,
    len: usize,
}

#[derive(Debug, Default)]
pub struct Bvh {
    nodes: Vec<BvhNode>,
    indices: Vec<usize>,
}

impl Bvh {
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn build(triangles: &[Triangle]) -> Self {
        let mut bvh = Self {
            nodes: Vec::with_capacity(triangles.len().saturating_mul(2)),
            indices: (0..triangles.len()).collect(),
        };
        if !triangles.is_empty() {
            bvh.build_node(0, triangles.len(), triangles);
        }
        bvh
    }

    fn build_node(&mut self, start: usize, len: usize, triangles: &[Triangle]) -> usize {
        let node_index = self.nodes.len();
        let (min, max) = self.bounds(start, len, triangles);
        self.nodes.push(BvhNode {
            min,
            max,
            start,
            len,
            ..BvhNode::default()
        });

        if len <= LEAF_TRIANGLES {
            return node_index;
        }

        let (centroid_min, centroid_max) = self.centroid_bounds(start, len, triangles);
        let extent = centroid_max - centroid_min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };

        self.indices[start..start + len].sort_by(|left, right| {
            let a = triangles[*left].centroid[axis];
            let b = triangles[*right].centroid[axis];
            a.partial_cmp(&b).unwrap_or(Ordering::Equal)
        });

        let mid = start + len / 2;
        if mid == start || mid == start + len {
            return node_index;
        }

        let left = self.build_node(start, mid - start, triangles);
        let right = self.build_node(mid, start + len - mid, triangles);
        self.nodes[node_index].left = Some(left);
        self.nodes[node_index].right = Some(right);
        self.nodes[node_index].len = 0;
        node_index
    }

    fn bounds(&self, start: usize, len: usize, triangles: &[Triangle]) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for &tri_index in &self.indices[start..start + len] {
            let tri = triangles[tri_index];
            min = min.min(tri.min);
            max = max.max(tri.max);
        }
        (min, max)
    }

    fn centroid_bounds(&self, start: usize, len: usize, triangles: &[Triangle]) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for &tri_index in &self.indices[start..start + len] {
            let c = triangles[tri_index].centroid;
            min = min.min(c);
            max = max.max(c);
        }
        (min, max)
    }

    pub fn raycast(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_range: f32,
        triangles: &[Triangle],
    ) -> Option<(Vec3, f32)> {
        if self.nodes.is_empty() {
            return None;
        }

        let mut closest = max_range;
        let mut hit = None;
        let mut stack = vec![0usize];

        while let Some(node_index) = stack.pop() {
            let node = self.nodes[node_index];
            if !intersect_aabb(origin, direction, node.min, node.max, closest) {
                continue;
            }

            if let (Some(left), Some(right)) = (node.left, node.right) {
                stack.push(left);
                stack.push(right);
                continue;
            }

            for &tri_index in &self.indices[node.start..node.start + node.len] {
                if let Some(t) = intersect_triangle(origin, direction, triangles[tri_index]) {
                    if t > 0.0 && t < closest {
                        closest = t;
                        hit = Some(origin + direction * t);
                    }
                }
            }
        }

        hit.map(|p| (p, closest))
    }
}

fn intersect_aabb(origin: Vec3, direction: Vec3, min: Vec3, max: Vec3, max_t: f32) -> bool {
    let mut t_min = 0.0;
    let mut t_max = max_t;
    for axis in 0..3 {
        let o = origin[axis];
        let d = direction[axis];
        let min_axis = min[axis];
        let max_axis = max[axis];
        if d.abs() < RAY_EPSILON {
            if o < min_axis || o > max_axis {
                return false;
            }
            continue;
        }
        let inv = 1.0 / d;
        let mut t0 = (min_axis - o) * inv;
        let mut t1 = (max_axis - o) * inv;
        if t0 > t1 {
            std::mem::swap(&mut t0, &mut t1);
        }
        t_min = f32::max(t_min, t0);
        t_max = f32::min(t_max, t1);
        if t_max < t_min {
            return false;
        }
    }
    true
}

fn intersect_triangle(origin: Vec3, direction: Vec3, tri: Triangle) -> Option<f32> {
    let edge1 = tri.b - tri.a;
    let edge2 = tri.c - tri.a;
    let h = direction.cross(edge2);
    let det = edge1.dot(h);
    if det.abs() < RAY_EPSILON {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = origin - tri.a;
    let u = inv_det * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(edge1);
    let v = inv_det * direction.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = inv_det * edge2.dot(q);
    (t > RAY_EPSILON).then_some(t)
}
