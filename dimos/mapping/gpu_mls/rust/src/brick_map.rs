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

//! GPU-resident sparse voxel map: 8x8x8 bricks found through an
//! open-addressing hash table from brick coordinate to pool slot, each brick
//! an index into a pool holding only the voxels that exist. Both pools grow as
//! the map does. A frame uploads only its points and sensor origin.

use std::sync::Arc;

use bytemuck::{Pod, Zeroable};

use crate::gpu::{groups_1d, Gpu, PassTimer};

pub type VoxelKey = (i32, i32, i32);

const BRICK_VOXELS: u64 = 512;
const MOMENT_WORDS: u64 = 10;
const STAMPS_PER_VOXEL: u64 = 3;
const FINE_WORDS: u64 = 2;
const VOXEL_DATA_WORDS: u64 = 4;
const LISTS: u64 = 6;
const COUNTERS: u64 = 9;
const C_OVERFLOW: usize = 7;
const C_VOXELS: usize = 8;
/// Indirect argument slots: the lists, the brick pool's sites, then its voxels.
const DISPATCH_SLOTS: u64 = LISTS + 2;
const POOL_DISPATCH_SLOT: u64 = LISTS;
const VOXEL_DISPATCH_SLOT: u64 = LISTS + 1;
const L_NEW_VOXELS: u64 = 5;
/// Pool sizes before the map first grows them.
const INITIAL_BRICKS: u32 = 1 << 12;
const INITIAL_VOXELS: u32 = 1 << 18;
const FIXED_SCALE: f64 = (1u64 << 16) as f64;
const EMPTY_KEY: u32 = u32::MAX;
const S_EXISTS: u32 = 1;
const S_STALE: u32 = 2;
const S_PLANAR: u32 = 8;
/// Brick coordinates the 32-bit table key can hold, per axis, half-open.
const BRICK_RANGE_XY: (i32, i32) = (-1024, 1023);
const BRICK_RANGE_Z: (i32, i32) = (-512, 511);

const MODE_TRACE: u32 = 0;
const MODE_SEED: u32 = 1;
const MODE_INGEST: u32 = 2;

/// The CPU ray tracer's update parameters, plus the GPU pool sizes.
#[derive(Clone, Debug)]
pub struct MapConfig {
    pub voxel_size: f32,
    pub fine_divisor: u32,
    pub max_range: f32,
    pub ray_subsample: u32,
    pub shadow_depth: f32,
    pub grace_depth: f32,
    pub min_health: i32,
    pub max_health: i32,
    pub graze_cos: f32,
    /// Most bricks the pool may grow to.
    pub brick_capacity: u32,
    /// Most voxels the pool may grow to.
    pub voxel_capacity: u32,
    /// Most points one frame may upload.
    pub max_points: u32,
    /// Entries in each per-frame work list (hits, fits, misses, ...).
    pub list_capacity: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MapParams {
    origin: [f32; 4],
    voxel_size: f32,
    inv_voxel: f32,
    shadow_depth: f32,
    grace_depth: f32,
    graze_cos: f32,
    min_health: i32,
    max_health: i32,
    fine_divisor: u32,
    point_count: u32,
    ray_step: u32,
    frame: u32,
    mode: u32,
    table_mask: u32,
    brick_capacity: u32,
    list_capacity: u32,
    trace_pass: u32,
    voxel_capacity: u32,
    pad: [u32; 3],
}

/// One voxel read back from the GPU, moments in meters about its center.
#[derive(Clone, Debug)]
pub struct VoxelSnapshot {
    pub key: VoxelKey,
    pub health: i32,
    pub num_pts: u32,
    pub sum: [f64; 3],
    /// xx, xy, xz, yy, yz, zz.
    pub m2: [f64; 6],
    pub fine: u64,
    pub stale: bool,
    pub normal: Option<[f32; 3]>,
}

/// Per-frame list overflow and pool use, read back on demand.
#[derive(Clone, Debug)]
pub struct MapStats {
    pub bricks: u32,
    pub hits: u32,
    pub fits: u32,
    pub misses: u32,
    pub changed: u32,
    pub voxels: u32,
    /// Bit per list that overflowed, 256 table full, 512 brick pool full,
    /// 1024 voxel pool full.
    pub overflow: u32,
}

pub(crate) struct MapBuffers {
    pub params: wgpu::Buffer,
    /// The params the second, miss-listing walk reads.
    pub miss_params: wgpu::Buffer,
    pub table_keys: wgpu::Buffer,
    pub table_slots: wgpu::Buffer,
    pub brick_coords: wgpu::Buffer,
    pub brick_voxels: wgpu::Buffer,
    pub counters: wgpu::Buffer,
    pub lists: wgpu::Buffer,
    pub points: wgpu::Buffer,
    pub voxel_state: wgpu::Buffer,
    pub stamps: wgpu::Buffer,
    pub fine: wgpu::Buffer,
    pub moments: wgpu::Buffer,
    pub voxel_data: wgpu::Buffer,
    pub dispatch: wgpu::Buffer,
}

impl MapBuffers {
    /// The storage bindings in shader order, after the params uniform.
    fn storage(&self) -> [&wgpu::Buffer; 12] {
        [
            &self.table_keys,
            &self.table_slots,
            &self.brick_coords,
            &self.brick_voxels,
            &self.counters,
            &self.lists,
            &self.points,
            &self.voxel_state,
            &self.stamps,
            &self.fine,
            &self.moments,
            &self.voxel_data,
        ]
    }
}

struct Pipelines {
    prepare: wgpu::ComputePipeline,
    insert_bricks: wgpu::ComputePipeline,
    assign_bricks: wgpu::ComputePipeline,
    reserve_voxels: wgpu::ComputePipeline,
    assign_voxels: wgpu::ComputePipeline,
    rehash: wgpu::ComputePipeline,
    mark_hits: wgpu::ComputePipeline,
    apply_hits: wgpu::ComputePipeline,
    accumulate: wgpu::ComputePipeline,
    settle_hits: wgpu::ComputePipeline,
    apply_misses: wgpu::ComputePipeline,
    mark_stale: wgpu::ComputePipeline,
    trace_rays: wgpu::ComputePipeline,
    fit_normals: wgpu::ComputePipeline,
}

pub struct BrickMap {
    gpu: Arc<Gpu>,
    pub config: MapConfig,
    pub(crate) buffers: MapBuffers,
    pub(crate) layout: wgpu::BindGroupLayout,
    pub(crate) bind_group: wgpu::BindGroup,
    miss_bind_group: wgpu::BindGroup,
    dispatch_group: wgpu::BindGroup,
    dispatch_layout: wgpu::BindGroupLayout,
    pipelines: Pipelines,
    table_mask: u32,
    /// Allocated pool sizes, which grow toward the config's capacities.
    brick_pool: u32,
    voxel_pool: u32,
    /// Upper bounds on the pools' use: the last read counts plus one brick
    /// and one voxel per point uploaded since.
    brick_bound: u64,
    voxel_bound: u64,
    frame: u32,
    pub timer: Option<PassTimer>,
}

pub(crate) const MAP_COMMON: &str = include_str!("shaders/map_common.wgsl");

/// The 13 map bindings: params uniform, then the storage buffers in order.
pub(crate) const MAP_BINDINGS: u32 = 13;

impl BrickMap {
    /// Bytes each voxel holds across the per-voxel buffers.
    pub const VOXEL_BYTES: u64 =
        4 * (1 + STAMPS_PER_VOXEL + FINE_WORDS + MOMENT_WORDS + VOXEL_DATA_WORDS);

    fn largest_binding(gpu: &Gpu) -> u64 {
        let limits = gpu.device.limits();
        (limits.max_storage_buffer_binding_size as u64).min(limits.max_buffer_size)
    }

    /// The most bricks whose voxel index fits one storage binding on this device.
    pub fn max_brick_capacity(gpu: &Gpu) -> u32 {
        (Self::largest_binding(gpu) / (4 * BRICK_VOXELS)).min(u32::MAX as u64 / BRICK_VOXELS) as u32
    }

    /// The most voxels whose widest buffer, the moments, fits one storage
    /// binding on this device.
    pub fn max_voxel_capacity(gpu: &Gpu) -> u32 {
        (Self::largest_binding(gpu) / (4 * MOMENT_WORDS)).min(u32::MAX as u64 - 1) as u32
    }

    /// Half-full at `bricks`, plus room for the slotless keys of the frame
    /// that fills the pool (at most one per point).
    fn table_entries(bricks: u64, max_points: u32) -> u64 {
        (2 * bricks + max_points as u64).next_power_of_two()
    }

    /// Fails, without touching the device, when a buffer the config's
    /// capacities could grow to exceeds this device's binding or buffer limits.
    pub fn new(gpu: Arc<Gpu>, config: MapConfig) -> Result<Self, String> {
        let list_cap = config.list_capacity as u64;
        let largest = Self::largest_binding(&gpu);
        for (label, bytes) in [
            (
                "brick voxels",
                4 * BRICK_VOXELS * config.brick_capacity as u64,
            ),
            ("moments", 4 * MOMENT_WORDS * config.voxel_capacity as u64),
            ("lists", 4 * LISTS * list_cap),
            ("points", 16 * config.max_points as u64),
            (
                "table",
                4 * Self::table_entries(config.brick_capacity as u64, config.max_points),
            ),
        ] {
            if bytes > largest {
                return Err(format!(
                    "brick map {label} buffer needs {bytes} bytes but {} allows {largest} per binding; \
                     lower brick_capacity to at most {} and voxel_capacity to at most {} (or max_points / list_capacity)",
                    gpu.adapter_info.name,
                    Self::max_brick_capacity(&gpu),
                    Self::max_voxel_capacity(&gpu)
                ));
            }
        }
        let brick_pool = config.brick_capacity.min(INITIAL_BRICKS);
        let voxel_pool = config.voxel_capacity.min(INITIAL_VOXELS);
        let table_entries = Self::table_entries(brick_pool as u64, config.max_points);
        let bricks = brick_pool as u64;
        let voxels = voxel_pool as u64;
        let buffers = MapBuffers {
            params: gpu.uniform("map params", std::mem::size_of::<MapParams>() as u64),
            miss_params: gpu.uniform("miss params", std::mem::size_of::<MapParams>() as u64),
            table_keys: gpu.storage("table keys", 4 * table_entries),
            table_slots: gpu.storage("table slots", 4 * table_entries),
            brick_coords: gpu.storage("brick coords", 16 * bricks),
            brick_voxels: gpu.storage("brick voxels", 4 * BRICK_VOXELS * bricks),
            counters: gpu.storage("counters", 4 * COUNTERS),
            lists: gpu.storage("lists", 4 * LISTS * list_cap),
            points: gpu.storage("points", 16 * config.max_points as u64),
            voxel_state: gpu.storage("voxel state", 4 * voxels),
            stamps: gpu.storage("stamps", 4 * STAMPS_PER_VOXEL * voxels),
            fine: gpu.storage("fine", 4 * FINE_WORDS * voxels),
            moments: gpu.storage("moments", 4 * MOMENT_WORDS * voxels),
            voxel_data: gpu.storage("voxel data", 4 * VOXEL_DATA_WORDS * voxels),
            dispatch: gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("dispatch args"),
                size: 12 * DISPATCH_SLOTS,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::INDIRECT,
                mapped_at_creation: false,
            }),
        };
        gpu.queue.write_buffer(
            &buffers.table_keys,
            0,
            bytemuck::cast_slice(&vec![EMPTY_KEY; table_entries as usize]),
        );

        let layout = gpu.layout("map", MAP_BINDINGS, &[0], false);
        let dispatch_layout = gpu.layout("dispatch", 1, &[], false);
        let bind_group = Self::map_group(&gpu, &layout, &buffers, &buffers.params);
        let miss_bind_group = Self::map_group(&gpu, &layout, &buffers, &buffers.miss_params);
        let dispatch_group = gpu.bind_group("dispatch", &dispatch_layout, &[&buffers.dispatch]);

        let ingest = gpu.module(
            "ingest",
            &[MAP_COMMON, include_str!("shaders/ingest.wgsl")].concat(),
        );
        let trace = gpu.module(
            "trace",
            &[MAP_COMMON, include_str!("shaders/raytrace.wgsl")].concat(),
        );
        let fit = gpu.module(
            "fit",
            &[MAP_COMMON, include_str!("shaders/fit.wgsl")].concat(),
        );
        let map_only = [&layout];
        let pipelines = Pipelines {
            prepare: gpu.pipeline(&ingest, "prepare_dispatch", &[&layout, &dispatch_layout]),
            insert_bricks: gpu.pipeline(&ingest, "insert_bricks", &map_only),
            assign_bricks: gpu.pipeline(&ingest, "assign_bricks", &map_only),
            reserve_voxels: gpu.pipeline(&ingest, "reserve_voxels", &map_only),
            assign_voxels: gpu.pipeline(&ingest, "assign_voxels", &map_only),
            rehash: gpu.pipeline(&ingest, "rehash", &map_only),
            mark_hits: gpu.pipeline(&ingest, "mark_hits", &map_only),
            apply_hits: gpu.pipeline(&ingest, "apply_hits", &map_only),
            accumulate: gpu.pipeline(&ingest, "accumulate", &map_only),
            settle_hits: gpu.pipeline(&ingest, "settle_hits", &map_only),
            apply_misses: gpu.pipeline(&ingest, "apply_misses", &map_only),
            mark_stale: gpu.pipeline(&ingest, "mark_stale", &map_only),
            trace_rays: gpu.pipeline(&trace, "trace_rays", &map_only),
            fit_normals: gpu.pipeline(&fit, "fit_normals", &map_only),
        };

        Ok(Self {
            gpu,
            config,
            buffers,
            layout,
            bind_group,
            miss_bind_group,
            dispatch_group,
            dispatch_layout,
            pipelines,
            table_mask: (table_entries - 1) as u32,
            brick_pool,
            voxel_pool,
            brick_bound: 0,
            voxel_bound: 0,
            frame: 0,
            timer: None,
        })
    }

    fn map_group(
        gpu: &Gpu,
        layout: &wgpu::BindGroupLayout,
        buffers: &MapBuffers,
        params: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        let all: Vec<&wgpu::Buffer> = std::iter::once(params).chain(buffers.storage()).collect();
        gpu.bind_group("map", layout, &all)
    }

    fn rebind(&mut self) {
        let b = &self.buffers;
        self.bind_group = Self::map_group(&self.gpu, &self.layout, b, &b.params);
        self.miss_bind_group = Self::map_group(&self.gpu, &self.layout, b, &b.miss_params);
        self.dispatch_group =
            self.gpu
                .bind_group("dispatch", &self.dispatch_layout, &[&b.dispatch]);
    }

    /// Grows the pools, before a frame of `points` uploads, so it cannot fill
    /// them. Reads the counters back only when the running bounds say it might.
    fn reserve(&mut self, points: u32) {
        let n = points as u64;
        let fits = |bound: u64, pool: u32| bound + n <= pool as u64;
        if !(fits(self.brick_bound, self.brick_pool) && fits(self.voxel_bound, self.voxel_pool)) {
            let stats = self.stats();
            self.brick_bound = stats.bricks.min(self.brick_pool) as u64;
            self.voxel_bound = stats.voxels.min(self.voxel_pool) as u64;
            let grown = |bound: u64, pool: u32, cap: u32| {
                ((pool as u64 * 3 / 2).max(bound + n)).min(cap as u64) as u32
            };
            if !fits(self.brick_bound, self.brick_pool)
                && self.brick_pool < self.config.brick_capacity
            {
                self.grow_bricks(grown(
                    self.brick_bound,
                    self.brick_pool,
                    self.config.brick_capacity,
                ));
            }
            if !fits(self.voxel_bound, self.voxel_pool)
                && self.voxel_pool < self.config.voxel_capacity
            {
                self.grow_voxels(grown(
                    self.voxel_bound,
                    self.voxel_pool,
                    self.config.voxel_capacity,
                ));
            }
        }
        self.brick_bound += n;
        self.voxel_bound += n;
    }

    /// A buffer of `bytes` holding `old`'s contents, the rest zero.
    fn regrow(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        old: &wgpu::Buffer,
        label: &str,
        bytes: u64,
    ) -> wgpu::Buffer {
        let new = self.gpu.storage(label, bytes);
        encoder.copy_buffer_to_buffer(old, 0, &new, 0, old.size());
        new
    }

    fn grow_bricks(&mut self, pool: u32) {
        let bricks = pool as u64;
        let mut encoder = self.gpu.device.create_command_encoder(&Default::default());
        let b = &self.buffers;
        let coords = self.regrow(&mut encoder, &b.brick_coords, "brick coords", 16 * bricks);
        let index = self.regrow(
            &mut encoder,
            &b.brick_voxels,
            "brick voxels",
            4 * BRICK_VOXELS * bricks,
        );
        self.gpu.queue.submit([encoder.finish()]);
        self.buffers.brick_coords = coords;
        self.buffers.brick_voxels = index;
        self.brick_pool = pool;
        let entries = Self::table_entries(bricks, self.config.max_points);
        if entries > self.table_mask as u64 + 1 {
            let gpu = Arc::clone(&self.gpu);
            self.buffers.table_keys = gpu.storage("table keys", 4 * entries);
            self.buffers.table_slots = gpu.storage("table slots", 4 * entries);
            gpu.queue.write_buffer(
                &self.buffers.table_keys,
                0,
                bytemuck::cast_slice(&vec![EMPTY_KEY; entries as usize]),
            );
            self.table_mask = (entries - 1) as u32;
            self.rebind();
            let params = MapParams {
                table_mask: self.table_mask,
                brick_capacity: self.brick_pool,
                ..self.params((0.0, 0.0, 0.0), 0, MODE_INGEST)
            };
            gpu.queue
                .write_buffer(&self.buffers.params, 0, bytemuck::bytes_of(&params));
            let mut encoder = gpu.device.create_command_encoder(&Default::default());
            {
                let mut pass = encoder.begin_compute_pass(&Default::default());
                pass.set_bind_group(0, &self.bind_group, &[]);
                self.dispatch(&mut pass, &self.pipelines.rehash, groups_1d(pool));
            }
            gpu.queue.submit([encoder.finish()]);
        } else {
            self.rebind();
        }
    }

    fn grow_voxels(&mut self, pool: u32) {
        let voxels = pool as u64;
        let mut encoder = self.gpu.device.create_command_encoder(&Default::default());
        let b = &self.buffers;
        let state = self.regrow(&mut encoder, &b.voxel_state, "voxel state", 4 * voxels);
        let stamps = self.regrow(
            &mut encoder,
            &b.stamps,
            "stamps",
            4 * STAMPS_PER_VOXEL * voxels,
        );
        let fine = self.regrow(&mut encoder, &b.fine, "fine", 4 * FINE_WORDS * voxels);
        let moments = self.regrow(
            &mut encoder,
            &b.moments,
            "moments",
            4 * MOMENT_WORDS * voxels,
        );
        let data = self.regrow(
            &mut encoder,
            &b.voxel_data,
            "voxel data",
            4 * VOXEL_DATA_WORDS * voxels,
        );
        self.gpu.queue.submit([encoder.finish()]);
        let b = &mut self.buffers;
        (b.voxel_state, b.stamps, b.fine, b.moments, b.voxel_data) =
            (state, stamps, fine, moments, data);
        self.voxel_pool = pool;
        self.rebind();
    }

    pub fn gpu(&self) -> &Arc<Gpu> {
        &self.gpu
    }

    /// One lidar frame: hits, ray-traced clearing, health and moments. The
    /// same filter as the CPU update_map runs while packing the upload.
    pub fn integrate(&mut self, origin: (f32, f32, f32), points: &[(f32, f32, f32)]) {
        let packed = self.pack(points, Some(origin));
        self.run(origin, &packed, MODE_TRACE);
    }

    /// `integrate` without the clearing walk: occupancy, health and moments only.
    pub fn ingest(&mut self, origin: (f32, f32, f32), points: &[(f32, f32, f32)]) {
        let packed = self.pack(points, Some(origin));
        self.run(origin, &packed, MODE_INGEST);
    }

    /// The CPU seed_points: create absent voxels at seed health, without rays.
    pub fn seed(&mut self, points: &[(f32, f32, f32)]) {
        let packed = self.pack(points, None);
        self.run((0.0, 0.0, 0.0), &packed, MODE_SEED);
    }

    /// Points the map can hold, as vec4s. With an origin, also the CPU's
    /// nonzero and max-range filter.
    fn pack(&self, points: &[(f32, f32, f32)], origin: Option<(f32, f32, f32)>) -> Vec<[f32; 4]> {
        let inv = 1.0 / self.config.voxel_size;
        let max_range_sq = if self.config.max_range > 0.0 {
            self.config.max_range * self.config.max_range
        } else {
            f32::INFINITY
        };
        let brick_ok = |c: f32, (lo, hi): (i32, i32)| {
            let b = ((c * inv).floor() as i32) >> 3;
            (c * inv).is_finite() && b >= lo && b < hi
        };
        points
            .iter()
            .filter(|&&(x, y, z)| {
                if !(brick_ok(x, BRICK_RANGE_XY)
                    && brick_ok(y, BRICK_RANGE_XY)
                    && brick_ok(z, BRICK_RANGE_Z))
                {
                    return false;
                }
                origin.is_none_or(|o| {
                    let (dx, dy, dz) = (x - o.0, y - o.1, z - o.2);
                    let d2 = dx * dx + dy * dy + dz * dz;
                    d2 > 0.0 && d2 <= max_range_sq
                })
            })
            .map(|&(x, y, z)| [x, y, z, 0.0])
            .collect()
    }

    fn params(&self, origin: (f32, f32, f32), point_count: u32, mode: u32) -> MapParams {
        let c = &self.config;
        MapParams {
            origin: [origin.0, origin.1, origin.2, 0.0],
            voxel_size: c.voxel_size,
            inv_voxel: 1.0 / c.voxel_size,
            shadow_depth: c.shadow_depth,
            grace_depth: c.grace_depth,
            graze_cos: c.graze_cos,
            min_health: c.min_health,
            max_health: c.max_health,
            fine_divisor: if c.fine_divisor >= 2 {
                c.fine_divisor
            } else {
                0
            },
            point_count,
            ray_step: c.ray_subsample.max(1),
            frame: self.frame,
            mode,
            table_mask: self.table_mask,
            brick_capacity: self.brick_pool,
            list_capacity: c.list_capacity,
            trace_pass: 0,
            voxel_capacity: self.voxel_pool,
            pad: [0; 3],
        }
    }

    fn run(&mut self, origin: (f32, f32, f32), packed: &[[f32; 4]], mode: u32) {
        assert!(
            packed.len() <= self.config.max_points as usize,
            "frame of {} points exceeds max_points {}",
            packed.len(),
            self.config.max_points
        );
        self.reserve(packed.len() as u32);
        self.frame += 1;
        let gpu = Arc::clone(&self.gpu);
        let params = self.params(origin, packed.len() as u32, mode);
        gpu.queue
            .write_buffer(&self.buffers.params, 0, bytemuck::bytes_of(&params));
        let miss_params = MapParams {
            trace_pass: 1,
            ..params
        };
        gpu.queue.write_buffer(
            &self.buffers.miss_params,
            0,
            bytemuck::bytes_of(&miss_params),
        );
        gpu.queue
            .write_buffer(&self.buffers.points, 0, bytemuck::cast_slice(packed));
        // List counters restart each frame; the brick count carries over.
        gpu.queue.write_buffer(
            &self.buffers.counters,
            4,
            bytemuck::cast_slice(&[0u32; LISTS as usize]),
        );

        let point_groups = groups_1d(packed.len() as u32);
        let rays = (packed.len() as u32).div_ceil(params.ray_step);
        let ray_groups = groups_1d(rays);
        let mut encoder = gpu.device.create_command_encoder(&Default::default());

        self.pass(&mut encoder, "bricks+hits", |map, pass| {
            map.dispatch(pass, &map.pipelines.insert_bricks, point_groups);
            map.prepare(pass);
            map.dispatch_list(pass, &map.pipelines.assign_bricks, 0);
            map.dispatch(pass, &map.pipelines.reserve_voxels, point_groups);
            map.prepare(pass);
            map.dispatch_list(pass, &map.pipelines.assign_voxels, L_NEW_VOXELS);
            map.dispatch(pass, &map.pipelines.mark_hits, point_groups);
            map.prepare(pass);
        });

        if mode == MODE_TRACE {
            self.pass(&mut encoder, "trace walk", |map, pass| {
                map.dispatch(pass, &map.pipelines.trace_rays, ray_groups);
                map.prepare(pass);
            });
            self.pass(&mut encoder, "trace fit", |map, pass| {
                map.dispatch_list(pass, &map.pipelines.fit_normals, 2);
            });
            self.pass(&mut encoder, "trace walk 2", |map, pass| {
                pass.set_bind_group(0, &map.miss_bind_group, &[]);
                map.dispatch(pass, &map.pipelines.trace_rays, ray_groups);
                map.prepare(pass);
            });
        }

        self.pass(&mut encoder, "update", |map, pass| {
            map.dispatch_list(pass, &map.pipelines.apply_hits, 1);
            map.dispatch(pass, &map.pipelines.accumulate, point_groups);
            map.dispatch_list(pass, &map.pipelines.settle_hits, 1);
            map.dispatch_list(pass, &map.pipelines.apply_misses, 3);
            map.prepare(pass);
            map.dispatch_list(pass, &map.pipelines.mark_stale, 4);
        });
        if let Some(timer) = &self.timer {
            timer.resolve(&mut encoder);
        }
        gpu.queue.submit([encoder.finish()]);
    }

    fn pass(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        label: &'static str,
        body: impl FnOnce(&Self, &mut wgpu::ComputePass),
    ) {
        let timestamp_writes = self.timer.as_mut().and_then(|t| t.next(label));
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some(label),
            timestamp_writes,
        });
        pass.set_bind_group(0, &self.bind_group, &[]);
        body(self, &mut pass);
    }

    fn dispatch(
        &self,
        pass: &mut wgpu::ComputePass,
        pipeline: &wgpu::ComputePipeline,
        (x, y): (u32, u32),
    ) {
        if x == 0 {
            return;
        }
        pass.set_pipeline(pipeline);
        pass.dispatch_workgroups(x, y, 1);
    }

    fn dispatch_list(
        &self,
        pass: &mut wgpu::ComputePass,
        pipeline: &wgpu::ComputePipeline,
        slot: u64,
    ) {
        pass.set_pipeline(pipeline);
        pass.dispatch_workgroups_indirect(&self.buffers.dispatch, 12 * slot);
    }

    /// Refresh the indirect sizes of every list and the brick pool.
    pub(crate) fn prepare(&self, pass: &mut wgpu::ComputePass) {
        pass.set_pipeline(&self.pipelines.prepare);
        pass.set_bind_group(1, &self.dispatch_group, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }

    /// One thread per voxel, sized on the GPU by the last `prepare`.
    pub(crate) fn dispatch_voxels(
        &self,
        pass: &mut wgpu::ComputePass,
        pipeline: &wgpu::ComputePipeline,
    ) {
        self.dispatch_list(pass, pipeline, VOXEL_DISPATCH_SLOT);
    }

    /// One thread per brick site, sized on the GPU by the last `prepare`.
    pub(crate) fn dispatch_pool(
        &self,
        pass: &mut wgpu::ComputePass,
        pipeline: &wgpu::ComputePipeline,
    ) {
        self.dispatch_list(pass, pipeline, POOL_DISPATCH_SLOT);
    }

    pub fn finish(&self) {
        self.gpu.wait();
    }

    pub fn stats(&self) -> MapStats {
        let c: Vec<u32> = self.gpu.read(&self.buffers.counters, COUNTERS as usize);
        MapStats {
            bricks: c[0],
            hits: c[2],
            fits: c[3],
            misses: c[4],
            changed: c[5],
            voxels: c[C_VOXELS],
            overflow: c[C_OVERFLOW],
        }
    }

    /// Every existing voxel. Reads the whole used pool back.
    pub fn snapshot(&self) -> Vec<VoxelSnapshot> {
        let stats = self.stats();
        let bricks = stats.bricks.min(self.brick_pool) as usize;
        let voxels = stats.voxels.min(self.voxel_pool) as usize;
        let gpu = &self.gpu;
        let coords: Vec<[i32; 4]> = gpu.read(&self.buffers.brick_coords, bricks);
        let state: Vec<u32> = gpu.read(&self.buffers.voxel_state, voxels);
        let data: Vec<u32> = gpu.read(&self.buffers.voxel_data, voxels * VOXEL_DATA_WORDS as usize);
        let fine: Vec<u32> = gpu.read(&self.buffers.fine, voxels * FINE_WORDS as usize);
        let moments: Vec<u32> = gpu.read(&self.buffers.moments, voxels * MOMENT_WORDS as usize);
        let word = |v: usize, w: usize| data[v * VOXEL_DATA_WORDS as usize + w];
        let vs = self.config.voxel_size as f64;
        let fixed = |v: usize, word: usize| {
            moments[v * MOMENT_WORDS as usize + 1 + word] as i32 as f64 / FIXED_SCALE
        };
        let mut out = Vec::new();
        for (v, &s) in state.iter().enumerate() {
            if s & S_EXISTS == 0 {
                continue;
            }
            let site = word(v, 3) as usize;
            let b = coords[site / BRICK_VOXELS as usize];
            let l = (site % BRICK_VOXELS as usize) as i32;
            let key = (
                (b[0] << 3) | (l >> 6),
                (b[1] << 3) | ((l >> 3) & 7),
                (b[2] << 3) | (l & 7),
            );
            out.push(VoxelSnapshot {
                key,
                health: word(v, 0) as i32,
                num_pts: moments[v * MOMENT_WORDS as usize],
                sum: [0, 1, 2].map(|w| fixed(v, w) * vs),
                m2: [3, 4, 5, 6, 7, 8].map(|w| fixed(v, w) * vs * vs),
                fine: (fine[2 * v] as u64) | ((fine[2 * v + 1] as u64) << 32),
                stale: s & S_STALE != 0,
                normal: (s & S_PLANAR != 0).then(|| unpack_normal(word(v, 2))),
            });
        }
        out
    }

    /// Bytes the map holds on the GPU.
    pub fn gpu_bytes(&self) -> u64 {
        self.buffers.storage().iter().map(|b| b.size()).sum()
    }
}

/// The shader's octahedral unpack, for readbacks.
pub(crate) fn unpack_normal(packed: u32) -> [f32; 3] {
    let snorm = |bits: u32| ((bits & 0xffff) as u16 as i16 as f32 / 32767.0).max(-1.0);
    let (qx, qy) = (snorm(packed), snorm(packed >> 16));
    let mut n = [qx, qy, 1.0 - qx.abs() - qy.abs()];
    let t = (-n[2]).max(0.0);
    n[0] += if n[0] >= 0.0 { -t } else { t };
    n[1] += if n[1] >= 0.0 { -t } else { t };
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    n.map(|c| c / len)
}
