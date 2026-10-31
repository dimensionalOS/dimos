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

//! MLS surface extraction on the GPU over a dense window of columns read
//! from the brick map: the standable test, exact bit-parallel round hole
//! closing per z slice, and jump-flood wall distance per connected surface
//! seen from above.

use std::sync::Arc;

use bytemuck::{Pod, Zeroable};

use crate::brick_map::{unpack_normal, BrickMap, VoxelKey, MAP_COMMON};
use crate::gpu::{groups_1d, Gpu, PassTimer};

/// The CPU has_support neighborhood: square radius and z tolerance, in cells.
const SUPPORT_RADIUS: i32 = 3;
const SUPPORT_Z_TOL: i32 = 3;
/// The CPU HOLE_SPAN_CELLS: empty columns a gap may span and still be a hole.
const HOLE_SPAN_CELLS: i32 = 4;
const SURFACE_BINDINGS: u32 = 13;
const PLANE_BINDINGS: u32 = 7;
/// Uniform offsets must be 256-byte aligned for dynamic binding.
const FLOOD_STRIDE: u64 = 256;
/// Flood steps a uniform slot is kept for: both directions of every power of two.
const FLOOD_SLOTS: u64 = 2 * 17;
const TILE: u32 = 8;
/// Flood seeds pack x and y in 16 bits each.
const MAX_WINDOW_EDGE: u32 = 1 << 16;

/// The MLS planner's surface parameters, in meters, judged against each
/// cell's plane rather than whole voxels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceConfig {
    /// Clear space a cell needs between its plane and the underside above.
    pub robot_height_m: f32,
    /// Hole closing radius; the closing itself runs in whole cells.
    pub closing_radius_m: f32,
    /// Largest height change two neighboring cells may link across.
    pub step_threshold_m: f32,
    /// Planes tilted further than this are not standable; 90 or more disables the gate.
    pub max_slope_deg: f32,
    /// Two stacked voxels whose planes are within this height are one surface.
    pub plane_merge_m: f32,
    /// ...and whose normals are within this angle.
    pub plane_merge_deg: f32,
}

impl SurfaceConfig {
    /// The MLS planner module's defaults. A Go2 or G1 walks about 30 degree
    /// slopes, but a 5 cm plane fit on grass reads well steeper than the
    /// ground under it (a 27 degree grassy hill cut off most of grassy_field
    /// at 30), so the gate sits at 45 and still turns down walls and
    /// railings; stairs are unaffected since their treads are flat. Lidar
    /// noise is 1 to 2 cm, so planes 3 cm apart on two stacked voxels are one
    /// floor straddling a grid line.
    pub fn mls_defaults() -> Self {
        Self {
            robot_height_m: 0.3,
            closing_radius_m: 0.3,
            step_threshold_m: 0.16,
            max_slope_deg: 45.0,
            plane_merge_m: 0.03,
            plane_merge_deg: 15.0,
        }
    }

    pub fn closing_cells(&self, voxel_size: f32) -> i32 {
        (self.closing_radius_m / voxel_size).ceil() as i32
    }

    pub fn step_cells(&self, voxel_size: f32) -> i32 {
        (self.step_threshold_m / voxel_size).floor() as i32
    }

    pub fn clearance_cells(&self, voxel_size: f32) -> i32 {
        (self.robot_height_m / voxel_size).ceil() as i32
    }

    /// Columns past a region whose surface it depends on: two closings, the
    /// support neighborhood and a hole's span.
    pub fn reach_cells(&self, voxel_size: f32) -> i32 {
        2 * self.closing_cells(voxel_size) + SUPPORT_RADIUS + HOLE_SPAN_CELLS
    }
}

/// A box of voxels: the min corner and the extent along each axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    pub min: VoxelKey,
    pub size: (u32, u32, u32),
}

impl Window {
    /// The box covering `keys`, padded by `pad` columns in x and y. None
    /// when `keys` is empty.
    pub fn covering(keys: impl IntoIterator<Item = VoxelKey>, pad: i32) -> Option<Self> {
        let mut bounds: Option<(VoxelKey, VoxelKey)> = None;
        for (x, y, z) in keys {
            let (lo, hi) = bounds.get_or_insert(((x, y, z), (x, y, z)));
            *lo = (lo.0.min(x), lo.1.min(y), lo.2.min(z));
            *hi = (hi.0.max(x), hi.1.max(y), hi.2.max(z));
        }
        let (lo, hi) = bounds?;
        let extent =
            |lo: i32, hi: i32, pad: i32| (hi as i64 - lo as i64 + 1 + 2 * pad as i64) as u32;
        Some(Self {
            min: (lo.0 - pad, lo.1 - pad, lo.2),
            size: (
                extent(lo.0, hi.0, pad),
                extent(lo.1, hi.1, pad),
                extent(lo.2, hi.2, 0),
            ),
        })
    }

    fn words_per_column(&self) -> u32 {
        self.size.2.div_ceil(32)
    }

    fn columns(&self) -> u64 {
        self.size.0 as u64 * self.size.1 as u64
    }

    fn cells(&self) -> u64 {
        self.columns() * self.size.2 as u64
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SurfaceParams {
    window_min: [i32; 4],
    size: [u32; 4],
    closing_cells: i32,
    step_cells: i32,
    hole_span_cells: i32,
    support_radius: i32,
    support_z_tol: i32,
    voxel_size: f32,
    robot_height_m: f32,
    step_m: f32,
    min_up: f32,
    merge_m: f32,
    merge_cos: f32,
    pad: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FloodStep {
    step: i32,
    a_to_b: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct PlaneInfo {
    planes: u32,
    cells: u32,
    capacity: u32,
    pad: u32,
}

struct Buffers {
    occupied: wgpu::Buffer,
    standable: wgpu::Buffer,
    grown: wgpu::Buffer,
    surface: wgpu::Buffer,
    wall: wgpu::Buffer,
    wall_distance: wgpu::Buffer,
    seeds_b: wgpu::Buffer,
    plane_info: wgpu::Buffer,
    plane_count: wgpu::Buffer,
    plane_bounds: wgpu::Buffer,
    plane_table: wgpu::Buffer,
    heights: wgpu::Buffer,
    cell_normals: wgpu::Buffer,
    planar: wgpu::Buffer,
}

/// Per-plane-cell buffers, regrown when a frame's planes need more room.
struct PlaneCells {
    cells: u64,
    group: wgpu::BindGroup,
}

struct Pipelines {
    build_occupancy: wgpu::ComputePipeline,
    fit_cells: wgpu::ComputePipeline,
    merge_columns: wgpu::ComputePipeline,
    standable_columns: wgpu::ComputePipeline,
    grow_columns: wgpu::ComputePipeline,
    compose_surface: wgpu::ComputePipeline,
    find_walls: wgpu::ComputePipeline,
    uf_init: wgpu::ComputePipeline,
    link_components: wgpu::ComputePipeline,
    flatten_components: wgpu::ComputePipeline,
    link_planes: wgpu::ComputePipeline,
    flatten_planes: wgpu::ComputePipeline,
    number_planes: wgpu::ComputePipeline,
    bound_planes: wgpu::ComputePipeline,
    init_plane_cells: wgpu::ComputePipeline,
    scatter_walls: wgpu::ComputePipeline,
    plane_flood_pass: wgpu::ComputePipeline,
    measure_walls: wgpu::ComputePipeline,
    export_planes: wgpu::ComputePipeline,
}

/// Surface cells read back from the GPU.
pub struct SurfaceResult {
    /// Occupied cells with robot clearance above, before closing.
    pub standable: Vec<VoxelKey>,
    /// Standable cells plus the filled holes.
    pub surface: Vec<VoxelKey>,
    /// Surface cells next to a wall, cliff or drop.
    pub walls: Vec<VoxelKey>,
    /// Per surface cell, in `surface` order: its height in meters.
    pub heights: Vec<f32>,
    /// Per surface cell: its plane's upward normal, None for filled or planeless cells.
    pub normals: Vec<Option<[f32; 3]>>,
}

/// One frame's plane split.
#[derive(Clone, Copy, Debug, Default)]
pub struct PlaneStats {
    /// Planes found; past the capacity of one per window column they overflow.
    pub planes: u32,
    /// Plane cells the flood covered: the sum of plane bounding boxes.
    pub plane_cells: u64,
}

pub struct SurfaceExtractor {
    gpu: Arc<Gpu>,
    pub window: Window,
    pub config: SurfaceConfig,
    closing_cells: i32,
    buffers: Buffers,
    plane_capacity: u32,
    empty_group: wgpu::BindGroup,
    group: wgpu::BindGroup,
    flood_group: wgpu::BindGroup,
    plane_layout: wgpu::BindGroupLayout,
    plane_cells: Option<PlaneCells>,
    pipelines: Pipelines,
    /// Which seed buffer holds the latest flood result.
    current_in_a: bool,
    pub plane_stats: PlaneStats,
    params: wgpu::Buffer,
    voxel_size: f32,
    /// The window the buffers were sized for; `retarget` takes any that fits it.
    capacity: Window,
}

impl SurfaceExtractor {
    /// Fails when the window's per-cell buffers exceed the device's
    /// binding limit, or its x, y extent does not fit the 16-bit flood seeds.
    pub fn new(map: &BrickMap, window: Window, config: SurfaceConfig) -> Result<Self, String> {
        let gpu = Arc::clone(map.gpu());
        let voxel_size = map.config.voxel_size;
        let bit_words = 4 * window.columns() * window.words_per_column() as u64;
        let cells = 4 * window.cells();
        let limits = gpu.device.limits();
        let largest = (limits.max_storage_buffer_binding_size as u64).min(limits.max_buffer_size);
        if window.size.0 > MAX_WINDOW_EDGE || window.size.1 > MAX_WINDOW_EDGE {
            return Err(format!(
                "surface window {:?} is wider than {MAX_WINDOW_EDGE} columns",
                window.size
            ));
        }
        if 2 * cells > largest {
            return Err(format!(
                "surface window {:?} needs {} bytes of labels but {} allows {largest} per binding",
                window.size,
                2 * cells,
                gpu.adapter_info.name
            ));
        }
        // Planes past one per column overflow; their cells get no wall distance.
        let plane_capacity = window.columns().min(u32::MAX as u64) as u32;
        let buffers = Buffers {
            occupied: gpu.storage("occupied", bit_words),
            standable: gpu.storage("standable", bit_words),
            grown: gpu.storage("grown", bit_words),
            surface: gpu.storage("surface", bit_words),
            wall: gpu.storage("wall", bit_words),
            wall_distance: gpu.storage("wall distance", cells),
            seeds_b: gpu.storage("seeds b", cells),
            plane_info: gpu.uniform("plane info", std::mem::size_of::<PlaneInfo>() as u64),
            plane_count: gpu.storage("plane count", 4),
            plane_bounds: gpu.storage("plane bounds", 16 * plane_capacity as u64),
            plane_table: gpu.storage("plane table", 16 * plane_capacity as u64),
            heights: gpu.storage("heights", cells),
            cell_normals: gpu.storage("cell normals", cells),
            planar: gpu.storage("planar", bit_words),
        };
        let params = gpu.uniform(
            "surface params",
            std::mem::size_of::<SurfaceParams>() as u64,
        );
        let seeds_a = gpu.storage("seeds a", cells);
        let labels = gpu.storage("labels", 2 * cells);
        let flood_buffer = gpu.uniform("flood steps", FLOOD_STRIDE * FLOOD_SLOTS);
        let mut flood_bytes = vec![0u8; (FLOOD_STRIDE * FLOOD_SLOTS) as usize];
        for slot in 0..FLOOD_SLOTS {
            let step = FloodStep {
                step: 1 << (slot / 2),
                a_to_b: (slot % 2) as u32,
            };
            let at = (slot * FLOOD_STRIDE) as usize;
            flood_bytes[at..at + 8].copy_from_slice(bytemuck::bytes_of(&step));
        }
        gpu.queue.write_buffer(&flood_buffer, 0, &flood_bytes);
        write_params(&gpu, &params, window, &config, voxel_size);

        let empty_layout = gpu.layout("empty", 0, &[], false);
        let surface_layout = gpu.layout("surface", SURFACE_BINDINGS, &[0], false);
        let plane_layout = gpu.layout("planes", PLANE_BINDINGS, &[0], false);
        let flood_layout = gpu
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("flood"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: wgpu::BufferSize::new(8),
                    },
                    count: None,
                }],
            });
        let b = &buffers;
        let group = gpu.bind_group(
            "surface",
            &surface_layout,
            &[
                &params,
                &b.occupied,
                &b.standable,
                &b.grown,
                &b.surface,
                &b.wall,
                &seeds_a,
                &b.seeds_b,
                &b.wall_distance,
                &labels,
                &b.heights,
                &b.cell_normals,
                &b.planar,
            ],
        );
        let flood_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("flood"),
            layout: &flood_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &flood_buffer,
                    offset: 0,
                    size: wgpu::BufferSize::new(8),
                }),
            }],
        });
        let empty_group = gpu.bind_group("empty", &empty_layout, &[]);

        let module = gpu.module(
            "surfaces",
            &[
                MAP_COMMON,
                include_str!("shaders/fit.wgsl"),
                include_str!("shaders/surfaces.wgsl"),
            ]
            .concat(),
        );
        let with_map = [&map.layout, &surface_layout, &flood_layout];
        let with_planes = [&empty_layout, &surface_layout, &flood_layout, &plane_layout];
        let column = |entry: &str| gpu.pipeline(&module, entry, &with_map);
        let plane = |entry: &str| gpu.pipeline(&module, entry, &with_planes);
        let pipelines = Pipelines {
            build_occupancy: column("build_occupancy"),
            fit_cells: column("fit_cells"),
            merge_columns: column("merge_columns"),
            standable_columns: column("standable_columns"),
            grow_columns: column("grow_columns"),
            compose_surface: column("compose_surface"),
            find_walls: plane("find_walls"),
            uf_init: plane("uf_init"),
            link_components: plane("link_components"),
            flatten_components: plane("flatten_components"),
            link_planes: plane("link_planes"),
            flatten_planes: plane("flatten_planes"),
            number_planes: plane("number_planes"),
            bound_planes: plane("bound_planes"),
            init_plane_cells: plane("init_plane_cells"),
            scatter_walls: plane("scatter_walls"),
            plane_flood_pass: plane("plane_flood_pass"),
            measure_walls: plane("measure_walls"),
            export_planes: plane("export_planes"),
        };
        let mut extractor = Self {
            gpu,
            window,
            config,
            closing_cells: config.closing_cells(voxel_size),
            buffers,
            plane_capacity,
            empty_group,
            group,
            flood_group,
            plane_layout,
            plane_cells: None,
            pipelines,
            current_in_a: true,
            plane_stats: PlaneStats::default(),
            params,
            voxel_size,
            capacity: window,
        };
        extractor.ensure_plane_cells(1);
        extractor.write_plane_info(0, 0);
        Ok(extractor)
    }

    /// Point the next `extract` at `window`, reusing the buffers. False, and
    /// unchanged, when the window needs more room than they were sized for.
    pub fn retarget(&mut self, window: Window) -> bool {
        let cap = self.capacity;
        let fits = window.size.0 <= MAX_WINDOW_EDGE
            && window.size.1 <= MAX_WINDOW_EDGE
            && window.cells() <= cap.cells()
            && window.columns() <= cap.columns()
            && window.columns() * window.words_per_column() as u64
                <= cap.columns() * cap.words_per_column() as u64;
        if fits {
            self.window = window;
            write_params(
                &self.gpu,
                &self.params,
                window,
                &self.config,
                self.voxel_size,
            );
        }
        fits
    }

    /// Room for `cells` plane cells, regrowing the buffers and their group.
    fn ensure_plane_cells(&mut self, cells: u64) {
        if self.plane_cells.as_ref().is_some_and(|p| p.cells >= cells) {
            return;
        }
        let cells = cells.max(1).next_power_of_two();
        let gpu = &self.gpu;
        let a = gpu.storage("plane cells a", 4 * cells);
        let b = gpu.storage("plane cells b", 4 * cells);
        let owner = gpu.storage("plane owner", 4 * cells);
        let bufs = &self.buffers;
        let group = gpu.bind_group(
            "planes",
            &self.plane_layout,
            &[
                &bufs.plane_info,
                &bufs.plane_count,
                &bufs.plane_bounds,
                &bufs.plane_table,
                &a,
                &b,
                &owner,
            ],
        );
        self.plane_cells = Some(PlaneCells { cells, group });
    }

    fn write_plane_info(&self, planes: u32, cells: u32) {
        self.gpu.queue.write_buffer(
            &self.buffers.plane_info,
            0,
            bytemuck::bytes_of(&PlaneInfo {
                planes,
                cells,
                capacity: self.plane_capacity,
                pad: 0,
            }),
        );
    }

    /// Dynamic offset of the flood step `step`, reading the current buffer.
    fn flood_offset(&self, step: u32) -> u32 {
        let slot = 2 * step.trailing_zeros() as u64 + self.current_in_a as u64;
        (slot * FLOOD_STRIDE) as u32
    }

    fn column_groups(&self) -> (u32, u32, u32) {
        (
            self.window.size.0.div_ceil(TILE),
            self.window.size.1.div_ceil(TILE),
            1,
        )
    }

    fn cell_groups(&self) -> (u32, u32, u32) {
        (
            self.window.size.0.div_ceil(TILE),
            self.window.size.1.div_ceil(TILE),
            self.window.size.2,
        )
    }

    /// Bind the map-reading layout, or with no map the plane layout.
    fn bind(&self, pass: &mut wgpu::ComputePass, map: Option<&BrickMap>, step: u32) {
        match map {
            Some(map) => pass.set_bind_group(0, &map.bind_group, &[]),
            None => {
                pass.set_bind_group(0, &self.empty_group, &[]);
                if let Some(p) = &self.plane_cells {
                    pass.set_bind_group(3, &p.group, &[]);
                }
            }
        }
        pass.set_bind_group(1, &self.group, &[]);
        pass.set_bind_group(2, &self.flood_group, &[self.flood_offset(step)]);
    }

    fn run(
        &self,
        pass: &mut wgpu::ComputePass,
        map: Option<&BrickMap>,
        pipeline: &wgpu::ComputePipeline,
        groups: (u32, u32, u32),
        step: u32,
    ) {
        if groups.0 == 0 || groups.1 == 0 || groups.2 == 0 {
            return;
        }
        self.bind(pass, map, step);
        pass.set_pipeline(pipeline);
        pass.dispatch_workgroups(groups.0, groups.1, groups.2);
    }

    /// Steps `start`, ..., 2, 1, then 1 again, the JFA+1 cleanup pass.
    fn flood_steps(start: u32) -> Vec<u32> {
        let mut steps: Vec<u32> =
            std::iter::successors(Some(start.max(1)), |s| (*s > 1).then_some(s / 2)).collect();
        steps.push(1);
        steps
    }

    /// Standable test, hole fill and wall distance. The plane split is read
    /// back between the two submits to size the wall-distance flood.
    pub fn extract(&mut self, map: &mut BrickMap) {
        let gpu = Arc::clone(&self.gpu);
        let mut timer = map.timer.take();
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        self.encode_surface(&mut encoder, map, timer.as_mut());
        self.encode_planes(&mut encoder, timer.as_mut());
        gpu.queue.submit([encoder.finish()]);

        let numbered = gpu.read::<u32>(&self.buffers.plane_count, 1)[0];
        let planes = numbered.min(self.plane_capacity);
        let bounds: Vec<[i32; 4]> = gpu.read(&self.buffers.plane_bounds, planes as usize);
        let mut table: Vec<[u32; 4]> = Vec::with_capacity(planes as usize);
        let mut cells = 0u64;
        let mut widest = 1u32;
        for [x0, y0, x1, y1] in &bounds {
            let (w, h) = ((x1 - x0 + 1) as u32, (y1 - y0 + 1) as u32);
            table.push([cells as u32, *x0 as u32, *y0 as u32, w]);
            cells += w as u64 * h as u64;
            widest = widest.max(w).max(h);
        }
        self.plane_stats = PlaneStats {
            planes: numbered,
            plane_cells: cells,
        };
        self.ensure_plane_cells(cells);
        gpu.queue
            .write_buffer(&self.buffers.plane_table, 0, bytemuck::cast_slice(&table));
        self.write_plane_info(planes, cells as u32);

        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        self.encode_wall_flood(&mut encoder, timer.as_mut(), cells as u32, widest);
        if let Some(timer) = &timer {
            timer.resolve(&mut encoder);
        }
        map.timer = timer;
        gpu.queue.submit([encoder.finish()]);
    }

    fn encode_surface(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        map: &BrickMap,
        mut timer: Option<&mut PassTimer>,
    ) {
        encoder.clear_buffer(&self.buffers.occupied, 0, None);
        encoder.clear_buffer(&self.buffers.grown, 0, None);
        encoder.clear_buffer(&self.buffers.planar, 0, None);
        {
            let mut pass = begin(encoder, "plane fit", &mut timer);
            pass.set_bind_group(0, &map.bind_group, &[]);
            map.prepare(&mut pass);
            self.bind(&mut pass, Some(map), 1);
            map.dispatch_voxels(&mut pass, &self.pipelines.build_occupancy);
            map.dispatch_pool(&mut pass, &self.pipelines.fit_cells);
        }
        {
            let mut pass = begin(encoder, "standable", &mut timer);
            self.run(
                &mut pass,
                Some(map),
                &self.pipelines.merge_columns,
                self.column_groups(),
                1,
            );
            self.run(
                &mut pass,
                Some(map),
                &self.pipelines.standable_columns,
                self.column_groups(),
                1,
            );
        }
        let mut pass = begin(encoder, "hole fill", &mut timer);
        if self.closing_cells > 0 {
            self.run(
                &mut pass,
                Some(map),
                &self.pipelines.grow_columns,
                self.column_groups(),
                1,
            );
        }
        self.run(
            &mut pass,
            Some(map),
            &self.pipelines.compose_surface,
            self.column_groups(),
            1,
        );
    }

    /// Wall cells, connected surfaces, their split into planes, and each
    /// plane's bounding box.
    fn encode_planes(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        mut timer: Option<&mut PassTimer>,
    ) {
        encoder.clear_buffer(&self.buffers.plane_count, 0, None);
        let mut pass = begin(encoder, "planes", &mut timer);
        let p = &self.pipelines;
        let columns = self.column_groups();
        self.run(&mut pass, None, &p.find_walls, columns, 1);
        for pipeline in [
            &p.uf_init,
            &p.link_components,
            &p.flatten_components,
            &p.link_planes,
            &p.flatten_planes,
            &p.number_planes,
            &p.bound_planes,
        ] {
            self.run(&mut pass, None, pipeline, columns, 1);
        }
    }

    fn encode_wall_flood(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        mut timer: Option<&mut PassTimer>,
        plane_cells: u32,
        widest: u32,
    ) {
        let mut pass = begin(encoder, "wall distance", &mut timer);
        let (gx, gy) = groups_1d(plane_cells);
        let plane_groups = (gx, gy, 1);
        self.current_in_a = true;
        self.run(
            &mut pass,
            None,
            &self.pipelines.init_plane_cells,
            plane_groups,
            1,
        );
        self.run(
            &mut pass,
            None,
            &self.pipelines.scatter_walls,
            self.column_groups(),
            1,
        );
        for step in Self::flood_steps(widest.next_power_of_two() / 2) {
            self.run(
                &mut pass,
                None,
                &self.pipelines.plane_flood_pass,
                plane_groups,
                step,
            );
            self.current_in_a = !self.current_in_a;
        }
        self.run(
            &mut pass,
            None,
            &self.pipelines.measure_walls,
            self.column_groups(),
            1,
        );
    }
}

fn write_params(
    gpu: &Gpu,
    params: &wgpu::Buffer,
    window: Window,
    config: &SurfaceConfig,
    voxel_size: f32,
) {
    gpu.queue.write_buffer(
        params,
        0,
        bytemuck::bytes_of(&SurfaceParams {
            window_min: [window.min.0, window.min.1, window.min.2, 0],
            size: [
                window.size.0,
                window.size.1,
                window.size.2,
                window.words_per_column(),
            ],
            closing_cells: config.closing_cells(voxel_size),
            step_cells: config.step_cells(voxel_size),
            hole_span_cells: HOLE_SPAN_CELLS,
            support_radius: SUPPORT_RADIUS,
            support_z_tol: SUPPORT_Z_TOL,
            voxel_size,
            robot_height_m: config.robot_height_m,
            step_m: config.step_threshold_m,
            min_up: if config.max_slope_deg >= 90.0 {
                -1.0
            } else {
                config.max_slope_deg.to_radians().cos()
            },
            merge_m: config.plane_merge_m,
            merge_cos: config.plane_merge_deg.to_radians().cos(),
            pad: 0,
        }),
    );
}

fn begin(
    encoder: &mut wgpu::CommandEncoder,
    label: &'static str,
    timer: &mut Option<&mut PassTimer>,
) -> wgpu::ComputePass<'static> {
    let timestamp_writes = timer.as_mut().and_then(|t| t.next(label));
    encoder
        .begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some(label),
            timestamp_writes,
        })
        .forget_lifetime()
}

impl SurfaceExtractor {
    fn read_bits(&self, buffer: &wgpu::Buffer) -> Vec<VoxelKey> {
        let w = self.window;
        let zw = w.words_per_column() as usize;
        let words: Vec<u32> = self.gpu.read(buffer, w.columns() as usize * zw);
        let mut out = Vec::new();
        for y in 0..w.size.1 as usize {
            for x in 0..w.size.0 as usize {
                let base = (y * w.size.0 as usize + x) * zw;
                for z in 0..w.size.2 as usize {
                    if (words[base + z / 32] >> (z % 32)) & 1 != 0 {
                        out.push((w.min.0 + x as i32, w.min.1 + y as i32, w.min.2 + z as i32));
                    }
                }
            }
        }
        out
    }

    /// The surface cells and their heights in meters, skipping the rest of `read`.
    pub fn read_heights(&self) -> (Vec<VoxelKey>, Vec<f32>) {
        let surface = self.read_bits(&self.buffers.surface);
        let heights: Vec<f32> = self
            .gpu
            .read(&self.buffers.heights, self.window.cells() as usize);
        let heights = surface.iter().map(|&k| heights[self.cell_of(k)]).collect();
        (surface, heights)
    }

    pub fn read(&self) -> SurfaceResult {
        let surface = self.read_bits(&self.buffers.surface);
        let cells = self.window.cells() as usize;
        let heights: Vec<f32> = self.gpu.read(&self.buffers.heights, cells);
        let packed: Vec<u32> = self.gpu.read(&self.buffers.cell_normals, cells);
        let planar: std::collections::HashSet<VoxelKey> =
            self.read_bits(&self.buffers.planar).into_iter().collect();
        SurfaceResult {
            standable: self.read_bits(&self.buffers.standable),
            heights: surface.iter().map(|&k| heights[self.cell_of(k)]).collect(),
            normals: surface
                .iter()
                .map(|&k| {
                    planar
                        .contains(&k)
                        .then(|| unpack_normal(packed[self.cell_of(k)]))
                })
                .collect(),
            surface,
            walls: self.read_bits(&self.buffers.wall),
        }
    }

    /// The fitted plane's upward normal at each of `keys`, None where the
    /// cell has no plane; the slope the standable stage judges.
    pub fn plane_normals(&self, keys: &[VoxelKey]) -> Vec<Option<[f32; 3]>> {
        let packed: Vec<u32> = self
            .gpu
            .read(&self.buffers.cell_normals, self.window.cells() as usize);
        let planar: std::collections::HashSet<VoxelKey> =
            self.read_bits(&self.buffers.planar).into_iter().collect();
        keys.iter()
            .map(|&k| {
                planar
                    .contains(&k)
                    .then(|| unpack_normal(packed[self.cell_of(k)]))
            })
            .collect()
    }

    /// Raw surface bitmask words, the readback a CPU consumer would pay.
    pub fn read_surface_words(&self) -> Vec<u32> {
        let w = self.window;
        self.gpu.read(
            &self.buffers.surface,
            w.columns() as usize * w.words_per_column() as usize,
        )
    }

    fn cell_of(&self, (x, y, z): VoxelKey) -> usize {
        let w = self.window;
        let (lx, ly, lz) = (
            (x - w.min.0) as usize,
            (y - w.min.1) as usize,
            (z - w.min.2) as usize,
        );
        (lz * w.size.1 as usize + ly) * w.size.0 as usize + lx
    }

    /// Wall distance in meters per surface cell, None on a plane with no wall.
    pub fn wall_distances(&self) -> Vec<(VoxelKey, Option<f32>)> {
        let dist: Vec<f32> = self
            .gpu
            .read(&self.buffers.wall_distance, self.window.cells() as usize);
        self.read_bits(&self.buffers.surface)
            .into_iter()
            .map(|k| (k, Some(dist[self.cell_of(k)]).filter(|d| *d >= 0.0)))
            .collect()
    }

    /// The plane id of every surface cell.
    pub fn plane_ids(&mut self) -> Vec<(VoxelKey, u32)> {
        let mut encoder = self.gpu.device.create_command_encoder(&Default::default());
        {
            let mut pass = begin(&mut encoder, "export planes", &mut None);
            self.run(
                &mut pass,
                None,
                &self.pipelines.export_planes,
                self.cell_groups(),
                1,
            );
        }
        self.gpu.queue.submit([encoder.finish()]);
        let ids: Vec<u32> = self
            .gpu
            .read(&self.buffers.seeds_b, self.window.cells() as usize);
        self.read_bits(&self.buffers.surface)
            .into_iter()
            .map(|k| (k, ids[self.cell_of(k)]))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::Window;

    #[test]
    fn empty_keys_have_no_window() {
        assert_eq!(Window::covering(std::iter::empty(), 4), None);
    }

    #[test]
    fn window_spans_the_extremes_without_overflow() {
        let w = Window::covering([(i32::MIN + 8, -3, 0), (i32::MAX - 8, 5, 2)], 4).unwrap();
        assert_eq!(w.min, (i32::MIN + 4, -7, 0));
        assert_eq!(w.size.1, 17);
        assert_eq!(w.size.2, 3);
    }
}
