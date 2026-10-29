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

//! Shared setup for the CPU-against-GPU parity tests.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};

use ahash::AHashMap;
use dimos_gpu_mls::brick_map::{BrickMap, MapConfig, VoxelKey, VoxelSnapshot};
use dimos_gpu_mls::gpu::Gpu;
use dimos_voxel_ray_tracing::voxel_ray_tracer::{Config, VoxelMap};

/// The ray tracer module's defaults at `voxel_size`.
pub fn cpu_config(voxel_size: f32) -> Config {
    Config {
        voxel_size,
        fine_divisor: 3,
        emit_fine: false,
        max_range: 30.0,
        ray_subsample: 1,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        graze_cos: 0.7,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        max_cloud_age_s: 0.0,
        tf_wait_timeout_s: 0.05,
        worker_threads: 8,
        region_m: 4.0,
        viz_cell_m: 4.0,
        viz_emit_every: 0,
        viz_sweep_cells: 0,
    }
}

pub fn gpu_config(cfg: &Config, brick_capacity: u32, max_points: u32) -> MapConfig {
    MapConfig {
        voxel_size: cfg.voxel_size,
        fine_divisor: cfg.fine_divisor,
        max_range: cfg.max_range,
        ray_subsample: cfg.ray_subsample,
        shadow_depth: cfg.shadow_depth,
        grace_depth: cfg.grace_depth,
        min_health: cfg.min_health,
        max_health: cfg.max_health,
        graze_cos: cfg.graze_cos,
        brick_capacity,
        voxel_capacity: brick_capacity
            .saturating_mul(512)
            .min(BrickMap::max_voxel_capacity(&gpu())),
        max_points,
        list_capacity: 1 << 22,
    }
}

static GPU: Mutex<Option<Arc<Gpu>>> = Mutex::new(None);

unsafe extern "C" {
    fn atexit(callback: extern "C" fn()) -> i32;
}

/// Drops the shared device before the driver unloads, whose worker thread
/// otherwise runs into the unloaded library and segfaults the exit.
extern "C" fn drop_gpu() {
    GPU.lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .take();
}

/// One device for the whole test binary: on Jetson, destroying a device while
/// another thread submits to its own deadlocks the driver.
pub fn gpu() -> Arc<Gpu> {
    let mut slot = GPU.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    slot.get_or_insert_with(|| {
        let gpu = Gpu::new().expect("these tests need a GPU");
        eprintln!(
            "adapter: {} ({:?})",
            gpu.adapter_info.name, gpu.adapter_info.backend
        );
        // Registered after the driver loaded, so it runs before the driver's own teardown.
        unsafe { atexit(drop_gpu) };
        gpu
    })
    .clone()
}

/// How a GPU map differs from a CPU one, voxel by voxel.
#[derive(Debug, Default)]
pub struct MapDiff {
    pub cpu_voxels: usize,
    pub gpu_voxels: usize,
    pub only_cpu: Vec<VoxelKey>,
    pub only_gpu: Vec<VoxelKey>,
    pub health: usize,
    pub count: usize,
    pub fine: usize,
    /// Largest moment error relative to the point count times the voxel scale.
    pub moment_rel_err: f64,
}

impl MapDiff {
    pub fn mismatched(&self) -> usize {
        self.only_cpu.len() + self.only_gpu.len() + self.health + self.count + self.fine
    }
}

pub fn diff_maps(cpu: &VoxelMap, gpu: &[VoxelSnapshot], voxel_size: f32) -> MapDiff {
    let by_key: AHashMap<VoxelKey, &VoxelSnapshot> = gpu.iter().map(|v| (v.key, v)).collect();
    let mut d = MapDiff {
        cpu_voxels: cpu.voxels.len(),
        gpu_voxels: gpu.len(),
        ..Default::default()
    };
    let vs = voxel_size as f64;
    for (key, v) in cpu.voxels.iter() {
        let Some(g) = by_key.get(&key) else {
            d.only_cpu.push(key);
            continue;
        };
        d.health += (v.health() != g.health) as usize;
        d.fine += (v.fine_mask() != g.fine) as usize;
        let (n, sum, m2) = v.moments();
        if n != g.num_pts {
            d.count += 1;
            continue;
        }
        let scale = n.max(1) as f64;
        for a in 0..3 {
            d.moment_rel_err = d
                .moment_rel_err
                .max((sum[a] as f64 - g.sum[a]).abs() / (scale * vs));
        }
        let pairs = [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)];
        for (i, &(r, c)) in pairs.iter().enumerate() {
            d.moment_rel_err = d
                .moment_rel_err
                .max((m2[(r, c)] as f64 - g.m2[i]).abs() / (scale * vs * vs));
        }
    }
    let cpu_keys: ahash::AHashSet<VoxelKey> = cpu.voxels.iter().map(|(k, _)| k).collect();
    d.only_gpu = gpu
        .iter()
        .map(|v| v.key)
        .filter(|k| !cpu_keys.contains(k))
        .collect();
    d
}

/// A real map cloud from the repo's LFS data, None when it is not pulled.
pub fn lfs_cloud(name: &str) -> Option<Vec<(f32, f32, f32)>> {
    let path = dimos_gpu_mls::data_dir().join(name);
    let bytes = std::fs::read(&path).ok()?;
    Some(decode_pointcloud2_xyz(&bytes))
}

/// The xyz float fields of an LCM-encoded sensor_msgs/PointCloud2.
fn decode_pointcloud2_xyz(bytes: &[u8]) -> Vec<(f32, f32, f32)> {
    use lcm_msgs::sensor_msgs::PointCloud2;
    let cloud = PointCloud2::decode(bytes).expect("a PointCloud2");
    let field = |name: &str| {
        cloud
            .fields
            .iter()
            .find(|f| f.name == name)
            .map(|f| f.offset as usize)
            .expect("xyz field")
    };
    let (ox, oy, oz) = (field("x"), field("y"), field("z"));
    let step = cloud.point_step as usize;
    let at = |p: usize, off: usize| {
        f32::from_le_bytes(cloud.data[p + off..p + off + 4].try_into().unwrap())
    };
    (0..(cloud.width * cloud.height) as usize)
        .map(|i| i * step)
        .map(|p| (at(p, ox), at(p, oy), at(p, oz)))
        .filter(|p| p.0.is_finite() && p.1.is_finite() && p.2.is_finite())
        .collect()
}
