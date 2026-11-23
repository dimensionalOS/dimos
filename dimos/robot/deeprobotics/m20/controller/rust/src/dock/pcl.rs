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

//! The PCL 1.10 filters reflective_column_node links (Ubuntu 20.04 libpcl_filters / libpcl_segmentation 1.10, GCC 9
//! libstdc++), on pcl::PointXYZ clouds as [x, y, z, 1] in f32: [`voxel_grid`] (VoxelGrid::applyFilter),
//! [`radius_outlier_removal`] (RadiusOutlierRemoval), [`euclidean_clusters`] (EuclideanClusterExtraction::extract on a
//! sorted KdTreeFLANN), [`centroid`] (compute3DCentroid) and [`min_max`] (getMinMax3D). The point order and float
//! arithmetic are the library's, so sums come out as its do; [`std_sort`] is libstdc++'s std::sort, since VoxelGrid's
//! and the cluster ordering's sorts are not stable.

use std::collections::HashMap;

/// A pcl::PointXYZ: x, y, z and the 1 in data[3].
pub type Point = [f32; 4];

const SORT_THRESHOLD: usize = 16;

/// libstdc++'s std::sort (GCC 9 bits/stl_algo.h): introsort, median-of-three pivot, heap sort past 2 log2(n) levels,
/// then a final insertion sort; elements that compare equal end in the order it leaves them.
pub fn std_sort<T: Copy>(v: &mut [T], less: impl Fn(&T, &T) -> bool) {
    let n = v.len();
    if n > 1 {
        let lg = (usize::BITS - 1 - n.leading_zeros()) as usize;
        introsort_loop(v, 0, n, 2 * lg, &less);
        final_insertion_sort(v, 0, n, &less);
    }
}

fn introsort_loop<T: Copy>(
    v: &mut [T],
    first: usize,
    mut last: usize,
    mut depth: usize,
    less: &impl Fn(&T, &T) -> bool,
) {
    while last - first > SORT_THRESHOLD {
        if depth == 0 {
            make_heap(v, first, last, less);
            sort_heap(v, first, last, less);
            return;
        }
        depth -= 1;
        let cut = unguarded_partition_pivot(v, first, last, less);
        introsort_loop(v, cut, last, depth, less);
        last = cut;
    }
}

fn unguarded_partition_pivot<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &impl Fn(&T, &T) -> bool,
) -> usize {
    let mid = first + (last - first) / 2;
    let (a, b, c) = (first + 1, mid, last - 1);
    // __move_median_to_first
    let m = if less(&v[a], &v[b]) {
        if less(&v[b], &v[c]) {
            b
        } else if less(&v[a], &v[c]) {
            c
        } else {
            a
        }
    } else if less(&v[a], &v[c]) {
        a
    } else if less(&v[b], &v[c]) {
        c
    } else {
        b
    };
    v.swap(first, m);
    // __unguarded_partition(first + 1, last, pivot = first)
    let (mut lo, mut hi) = (first + 1, last);
    loop {
        while less(&v[lo], &v[first]) {
            lo += 1;
        }
        hi -= 1;
        while less(&v[first], &v[hi]) {
            hi -= 1;
        }
        if lo >= hi {
            return lo;
        }
        v.swap(lo, hi);
        lo += 1;
    }
}

fn final_insertion_sort<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &impl Fn(&T, &T) -> bool,
) {
    if last - first > SORT_THRESHOLD {
        insertion_sort(v, first, first + SORT_THRESHOLD, less);
        for i in first + SORT_THRESHOLD..last {
            unguarded_linear_insert(v, i, less);
        }
    } else {
        insertion_sort(v, first, last, less);
    }
}

fn insertion_sort<T: Copy>(v: &mut [T], first: usize, last: usize, less: &impl Fn(&T, &T) -> bool) {
    for i in first + 1..last {
        if less(&v[i], &v[first]) {
            let val = v[i];
            v.copy_within(first..i, first + 1);
            v[first] = val;
        } else {
            unguarded_linear_insert(v, i, less);
        }
    }
}

fn unguarded_linear_insert<T: Copy>(v: &mut [T], mut last: usize, less: &impl Fn(&T, &T) -> bool) {
    let val = v[last];
    while less(&val, &v[last - 1]) {
        v[last] = v[last - 1];
        last -= 1;
    }
    v[last] = val;
}

fn make_heap<T: Copy>(v: &mut [T], first: usize, last: usize, less: &impl Fn(&T, &T) -> bool) {
    let len = last - first;
    if len < 2 {
        return;
    }
    let mut parent = (len - 2) / 2;
    loop {
        let value = v[first + parent];
        adjust_heap(v, first, parent, len, value, less);
        if parent == 0 {
            return;
        }
        parent -= 1;
    }
}

fn sort_heap<T: Copy>(v: &mut [T], first: usize, mut last: usize, less: &impl Fn(&T, &T) -> bool) {
    while last - first > 1 {
        last -= 1;
        // __pop_heap(first, last, last)
        let value = v[last];
        v[last] = v[first];
        adjust_heap(v, first, 0, last - first, value, less);
    }
}

fn adjust_heap<T: Copy>(
    v: &mut [T],
    first: usize,
    mut hole: usize,
    len: usize,
    value: T,
    less: &impl Fn(&T, &T) -> bool,
) {
    let top = hole;
    let mut child = hole;
    while child < (len - 1) / 2 {
        child = 2 * (child + 1);
        if less(&v[first + child], &v[first + child - 1]) {
            child -= 1;
        }
        v[first + hole] = v[first + child];
        hole = child;
    }
    if len & 1 == 0 && child == (len - 2) / 2 {
        child = 2 * (child + 1);
        v[first + hole] = v[first + child - 1];
        hole = child - 1;
    }
    // __push_heap
    while hole > top {
        let parent = (hole - 1) / 2;
        if !less(&v[first + parent], &value) {
            break;
        }
        v[first + hole] = v[first + parent];
        hole = parent;
    }
    v[first + hole] = value;
}

/// getMinMax3D on a dense cloud: element-wise min / max of all four lanes from (FLT_MAX, -FLT_MAX).
pub fn min_max(cloud: &[Point]) -> (Point, Point) {
    let (mut lo, mut hi) = ([f32::MAX; 4], [f32::MIN; 4]);
    for p in cloud {
        for k in 0..4 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

/// compute3DCentroid on a dense cloud: x, y, z summed in order in f32, each divided by the count; data[3] = 1.
pub fn centroid(cloud: &[Point]) -> Point {
    let mut s = [0.0f32; 3];
    for p in cloud {
        for k in 0..3 {
            s[k] += p[k];
        }
    }
    let n = cloud.len() as f32;
    [s[0] / n, s[1] / n, s[2] / n, 1.0]
}

/// VoxelGrid<PointXYZ>::applyFilter (downsample_all_data, no field filter, min_points_per_voxel 0): every point's
/// voxel index from its floor(p / leaf) off the cloud's minimum, the index list std::sort-ed, each voxel's points
/// averaged in that order (CentroidPoint: an f32 sum divided by the count), the voxels in index order.
pub fn voxel_grid(cloud: &[Point], leaf: [f32; 3]) -> Vec<Point> {
    let inv = [1.0 / leaf[0], 1.0 / leaf[1], 1.0 / leaf[2]];
    let (lo, hi) = min_max(cloud);
    let span = |k: usize| ((hi[k] - lo[k]) * inv[k]) as i64 + 1;
    // "Leaf size is too small for the input dataset. Integer indices would overflow": the cloud as it is
    if span(0).saturating_mul(span(1)).saturating_mul(span(2)) > i32::MAX as i64 {
        return cloud.to_vec();
    }
    let min_b = [0, 1, 2].map(|k| (lo[k] * inv[k]).floor() as i32);
    let max_b = [0, 1, 2].map(|k| (hi[k] * inv[k]).floor() as i32);
    let div = [0, 1, 2].map(|k| max_b[k] - min_b[k] + 1);
    let mul = [1, div[0], div[0].wrapping_mul(div[1])];
    // (voxel index, point index): cloud_point_index_idx, compared by the voxel index only
    let mut index: Vec<(u32, u32)> = cloud
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let ijk = [0, 1, 2].map(|k| ((p[k] * inv[k]).floor() - min_b[k] as f32) as i32);
            let idx = ijk[0]
                .wrapping_mul(mul[0])
                .wrapping_add(ijk[1].wrapping_mul(mul[1]))
                .wrapping_add(ijk[2].wrapping_mul(mul[2]));
            (idx as u32, i as u32)
        })
        .collect();
    std_sort(&mut index, |a, b| a.0 < b.0);
    let mut out = Vec::new();
    let mut first = 0;
    while first < index.len() {
        let mut last = first + 1;
        while last < index.len() && index[last].0 == index[first].0 {
            last += 1;
        }
        let mut s = [0.0f32; 3];
        for &(_, i) in &index[first..last] {
            for k in 0..3 {
                s[k] += cloud[i as usize][k];
            }
        }
        let n = (last - first) as f32;
        out.push([s[0] / n, s[1] / n, s[2] / n, 1.0]);
        first = last;
    }
    out
}

/// flann::L2_Simple<float> over x, y, z (the query minus the point, each square added on; GCC contracts the adds into
/// fused multiply-adds on aarch64).
fn sq_dist(a: &Point, b: &Point) -> f32 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    d[2].mul_add(d[2], d[1].mul_add(d[1], d[0] * d[0]))
}

/// A cell grid over a cloud for radius searches: every point within `radius` of a query is in the 27 cells around it
/// (cells a little wider than the radius, so rounding in the cell index never loses one).
struct Grid {
    cell: f64,
    cells: HashMap<[i64; 3], Vec<u32>>,
}

impl Grid {
    fn new(cloud: &[Point], radius: f64) -> Grid {
        let cell = radius * 1.5;
        let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
        for (i, p) in cloud.iter().enumerate() {
            cells.entry(Self::key(cell, p)).or_default().push(i as u32);
        }
        Grid { cell, cells }
    }

    fn key(cell: f64, p: &Point) -> [i64; 3] {
        [0, 1, 2].map(|k| (p[k] as f64 / cell).floor() as i64)
    }

    /// FLANN's radius search (KDTreeSingleIndex: dist < radius^2, the radius squared in f64 then cast to f32): the
    /// points within, in no particular order.
    fn within<'a>(
        &'a self,
        cloud: &'a [Point],
        q: &'a Point,
        r2: f32,
    ) -> impl Iterator<Item = usize> + 'a {
        let [x, y, z] = Self::key(self.cell, q);
        (x - 1..=x + 1)
            .flat_map(move |a| {
                (y - 1..=y + 1).flat_map(move |b| (z - 1..=z + 1).map(move |c| [a, b, c]))
            })
            .filter_map(|k| self.cells.get(&k))
            .flatten()
            .map(|&i| i as usize)
            .filter(move |&i| sq_dist(q, &cloud[i]) < r2)
    }
}

/// RadiusOutlierRemoval<PointXYZ> (radius search): a point with `min_neighbors` or fewer points within `radius`
/// (itself included) is dropped; the rest keep their order.
pub fn radius_outlier_removal(cloud: &[Point], radius: f64, min_neighbors: usize) -> Vec<Point> {
    let r2 = (radius * radius) as f32;
    let grid = Grid::new(cloud, radius);
    cloud
        .iter()
        .filter(|q| grid.within(cloud, q, r2).count() > min_neighbors)
        .copied()
        .collect()
}

/// EuclideanClusterExtraction<PointXYZ>::extract (pcl::search::KdTree sorted, so the query itself is the first
/// result and skipped): seeded in index order, each cluster grows by radius searches of `tolerance` (f32, squared in
/// f64) to every point it reaches; kept with min..=max points, its indices sorted; then std::sort over the reversed
/// list by size, the largest first.
pub fn euclidean_clusters(
    cloud: &[Point],
    tolerance: f32,
    min: usize,
    max: usize,
) -> Vec<Vec<usize>> {
    let radius = tolerance as f64;
    let r2 = (radius * radius) as f32;
    let grid = Grid::new(cloud, radius);
    let mut processed = vec![false; cloud.len()];
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    for i in 0..cloud.len() {
        if processed[i] {
            continue;
        }
        processed[i] = true;
        let mut queue = vec![i];
        let mut k = 0;
        while k < queue.len() {
            let q = queue[k];
            for j in grid.within(cloud, &cloud[q], r2) {
                if !processed[j] {
                    processed[j] = true;
                    queue.push(j);
                }
            }
            k += 1;
        }
        if (min..=max).contains(&queue.len()) {
            queue.sort_unstable();
            clusters.push(queue);
        }
    }
    // std::sort (clusters.rbegin (), clusters.rend (), comparePointClusters)
    let mut order: Vec<usize> = (0..clusters.len()).rev().collect();
    std_sort(&mut order, |a, b| clusters[*a].len() < clusters[*b].len());
    order.reverse();
    let mut slots: Vec<Option<Vec<usize>>> = clusters.into_iter().map(Some).collect();
    order.into_iter().filter_map(|i| slots[i].take()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Small deterministic PRNG (xorshift).
    fn rng(state: &mut u64) -> u64 {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        *state
    }

    #[test]
    fn std_sort_sorts_and_leaves_equal_keys_as_libstdcxx_does() {
        let mut s = 7u64;
        for n in [0, 1, 2, 5, 16, 17, 40, 300, 5000] {
            let mut v: Vec<(u32, u32)> = (0..n).map(|i| ((rng(&mut s) % 50) as u32, i)).collect();
            std_sort(&mut v, |a, b| a.0 < b.0);
            assert!(v.windows(2).all(|w| w[0].0 <= w[1].0), "n {n}");
            let mut ids: Vec<u32> = v.iter().map(|p| p.1).collect();
            ids.sort_unstable();
            assert_eq!(ids, (0..n).collect::<Vec<u32>>());
        }
        // libstdc++ (GCC 15's bits/stl_algo.h, the same introsort as GCC 9's) on these 20 (key, id) pairs by key, and a
        // hash of its order on 3000 keys 0-49 from the xorshift above seeded 7
        let mut v: Vec<(u32, u32)> = [3, 1, 3, 2, 1, 3, 0, 2, 2, 1, 3, 0, 1, 2, 3, 0, 1, 1, 2, 3]
            .iter()
            .enumerate()
            .map(|(i, &k)| (k, i as u32))
            .collect();
        std_sort(&mut v, |a, b| a.0 < b.0);
        let ids: Vec<u32> = v.iter().map(|p| p.1).collect();
        assert_eq!(
            ids,
            [6, 11, 15, 1, 4, 9, 17, 12, 16, 3, 18, 7, 8, 13, 19, 0, 14, 10, 5, 2]
        );
        let mut s = 7u64;
        let mut w: Vec<(u32, u32)> = (0..3000).map(|i| ((rng(&mut s) % 50) as u32, i)).collect();
        std_sort(&mut w, |a, b| a.0 < b.0);
        let hash = w.iter().fold(0u64, |h, p| {
            h.wrapping_mul(1000003).wrapping_add(p.1 as u64)
        });
        assert_eq!(hash, 17023255052605782972);
    }

    #[test]
    fn heap_sort_fallback_sorts() {
        // depth 0 from the start: every partition step goes to the heap sort
        let mut s = 3u64;
        let mut v: Vec<u32> = (0..200).map(|_| (rng(&mut s) % 1000) as u32).collect();
        let mut want = v.clone();
        want.sort_unstable();
        let n = v.len();
        make_heap(&mut v, 0, n, &|a: &u32, b: &u32| a < b);
        sort_heap(&mut v, 0, n, &|a: &u32, b: &u32| a < b);
        assert_eq!(v, want);
    }

    #[test]
    fn voxels_average_their_points_in_index_order() {
        let cloud = vec![
            [0.001, 0.001, 0.001, 1.0],
            [0.5, 0.5, 0.5, 1.0],
            [0.009, 0.003, 0.002, 1.0],
        ];
        let out = voxel_grid(&cloud, [0.01; 3]);
        assert_eq!(out.len(), 2);
        assert_eq!(
            out[0],
            [
                (0.001f32 + 0.009) / 2.0,
                (0.001f32 + 0.003) / 2.0,
                (0.001f32 + 0.002) / 2.0,
                1.0
            ]
        );
        assert_eq!(out[1], [0.5, 0.5, 0.5, 1.0]);
    }

    #[test]
    fn outliers_and_clusters_by_radius() {
        // a 5 x 5 patch 1 cm apart, a pair 2 m off
        let mut cloud: Vec<Point> = (0..25)
            .map(|i| [(i % 5) as f32 * 0.01, (i / 5) as f32 * 0.01, 0.0, 1.0])
            .collect();
        cloud.push([2.0, 0.0, 0.0, 1.0]);
        cloud.push([2.01, 0.0, 0.0, 1.0]);
        assert_eq!(radius_outlier_removal(&cloud, 0.3, 15).len(), 25);
        let clusters = euclidean_clusters(&cloud, 0.011, 2, 100);
        assert_eq!(clusters.len(), 2);
        assert_eq!(clusters[0], (0..25).collect::<Vec<usize>>());
        assert_eq!(clusters[1], vec![25, 26]);
        // under the 1 cm spacing nothing links
        assert!(euclidean_clusters(&cloud, 0.009, 2, 100).is_empty());
    }
}
