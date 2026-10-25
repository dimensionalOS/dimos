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

//! Stage 1, uniqueness: is this scan distinctive enough to place, in O(scan). Universal uniqueness is
//! measured on the scan alone: how many scan-context cells it fills (`similarity_discovery`'s
//! descriptor occupancy) and its geometric degeneracy here. Current-map uniqueness (how unlike the
//! rest of the map it is) is not measured yet.

use rayon::prelude::*;

use crate::mat3;
use crate::pointcloud::KdTree;

/// Geometric degeneracy (Zhang 2016 / X-ICP): the spread of the scan's surface normals, as the smallest
/// eigenvalue of their scatter matrix over its trace. Near 0 means every surface faces about the same way
/// (a corridor, a plane), which leaves ICP free to slide; 1/3 is the most spread. -1 for fewer than 20
/// points or normals.
pub fn degeneracy(cloud: &[[f32; 3]]) -> f32 {
    if cloud.len() < 20 {
        return -1.0;
    }
    let tree = KdTree::build(cloud);
    // each point's normal: the least-spread direction of its 10 nearest neighbours
    let normals: Vec<[f64; 3]> = cloud
        .par_iter()
        .filter_map(|p| {
            let neighbours = tree.knn(p, 10);
            if neighbours.len() < 3 {
                return None;
            }
            let n = neighbours.len() as f64;
            let mut mean = [0.0; 3];
            for &(i, _) in &neighbours {
                for k in 0..3 {
                    mean[k] += cloud[i][k] as f64 / n;
                }
            }
            let mut covariance = [[0.0; 3]; 3];
            for &(i, _) in &neighbours {
                let d = [0, 1, 2].map(|k| cloud[i][k] as f64 - mean[k]);
                for a in 0..3 {
                    for b in 0..3 {
                        covariance[a][b] += d[a] * d[b];
                    }
                }
            }
            let (_, vectors) = mat3::jacobi_eigen(covariance);
            Some([vectors[0][0], vectors[1][0], vectors[2][0]])
        })
        .collect();
    if normals.len() < 20 {
        return -1.0;
    }
    let mut scatter = [[0.0; 3]; 3];
    for n in &normals {
        for a in 0..3 {
            for b in 0..3 {
                scatter[a][b] += n[a] * n[b];
            }
        }
    }
    let (values, _) = mat3::jacobi_eigen(scatter);
    (values[0] / (values[0] + values[1] + values[2])) as f32
}
