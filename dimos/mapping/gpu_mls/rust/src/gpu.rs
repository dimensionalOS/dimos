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

//! Headless wgpu device plus the buffer, pipeline and readback helpers the
//! map and surface stages share.

use std::sync::{Arc, Mutex, OnceLock};

use bytemuck::Pod;

/// Workgroup size of every 1D kernel.
pub const GROUP_SIZE: u32 = 64;
/// Largest workgroup count along one dispatch dimension.
const MAX_GROUPS_PER_DIM: u32 = 65535;

pub struct Gpu {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub adapter_info: wgpu::AdapterInfo,
    /// Nanoseconds per timestamp tick, None without timestamp queries.
    pub timestamp_period: Option<f32>,
}

impl Gpu {
    /// The high-performance adapter with its full limits, headless.
    pub fn new() -> Result<Arc<Self>, String> {
        // One instance per process, with no GL backend. On Jetson, creating an
        // instance while another thread destroys one crashes the Vulkan loader,
        // and dropping a GL context while another thread drives Vulkan
        // deadlocks inside the shared NVIDIA driver.
        static INSTANCE: OnceLock<wgpu::Instance> = OnceLock::new();
        let instance = INSTANCE.get_or_init(|| {
            wgpu::Instance::new(
                wgpu::InstanceDescriptor {
                    backends: wgpu::Backends::PRIMARY,
                    ..wgpu::InstanceDescriptor::new_without_display_handle()
                }
                .with_env(),
            )
        });
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            ..Default::default()
        }))
        .map_err(|e| format!("no GPU adapter: {e}"))?;
        let timestamps = adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY);
        let info = adapter.get_info();
        let required_features = if timestamps {
            wgpu::Features::TIMESTAMP_QUERY
        } else {
            wgpu::Features::empty()
        };
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("gpu_mls"),
            required_features,
            required_limits: adapter.limits(),
            ..Default::default()
        }))
        .map_err(|e| format!("no GPU device: {e}"))?;
        let timestamp_period = timestamps.then(|| queue.get_timestamp_period());
        Ok(Arc::new(Self {
            device,
            queue,
            adapter_info: info,
            timestamp_period,
        }))
    }

    pub fn storage(&self, label: &str, bytes: u64) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.max(4).next_multiple_of(4),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    }

    pub fn uniform(&self, label: &str, bytes: u64) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    }

    /// A layout of `bindings` entries: uniforms at the given indices, the rest
    /// read-write storage.
    pub fn layout(
        &self,
        label: &str,
        bindings: u32,
        uniforms: &[u32],
        dynamic: bool,
    ) -> wgpu::BindGroupLayout {
        let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..bindings)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if uniforms.contains(&binding) {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage { read_only: false }
                    },
                    has_dynamic_offset: dynamic,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        self.device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some(label),
                entries: &entries,
            })
    }

    pub fn bind_group(
        &self,
        label: &str,
        layout: &wgpu::BindGroupLayout,
        buffers: &[&wgpu::Buffer],
    ) -> wgpu::BindGroup {
        let entries: Vec<wgpu::BindGroupEntry> = buffers
            .iter()
            .enumerate()
            .map(|(i, b)| wgpu::BindGroupEntry {
                binding: i as u32,
                resource: b.as_entire_binding(),
            })
            .collect();
        self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(label),
            layout,
            entries: &entries,
        })
    }

    pub fn module(&self, label: &str, source: &str) -> wgpu::ShaderModule {
        let _compiling = compile_lock();
        self.device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            })
    }

    pub fn pipeline(
        &self,
        module: &wgpu::ShaderModule,
        entry: &str,
        layouts: &[&wgpu::BindGroupLayout],
    ) -> wgpu::ComputePipeline {
        let _compiling = compile_lock();
        let layout = self
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some(entry),
                bind_group_layouts: &layouts.iter().map(|l| Some(*l)).collect::<Vec<_>>(),
                immediate_size: 0,
            });
        self.device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&layout),
                module,
                entry_point: Some(entry),
                // Shaders set their workgroup memory themselves. The automatic
                // zeroing is done by one thread per workgroup before any code
                // runs, which made the plane fit's 25 KB tile cost more than the
                // fit on large maps.
                compilation_options: wgpu::PipelineCompilationOptions {
                    zero_initialize_workgroup_memory: false,
                    ..Default::default()
                },
                cache: None,
            })
    }

    pub fn wait(&self) {
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("GPU poll failed");
    }

    /// Copy `count` elements from the start of `buffer` to the host.
    pub fn read<T: Pod>(&self, buffer: &wgpu::Buffer, count: usize) -> Vec<T> {
        let bytes = (count * std::mem::size_of::<T>()) as u64;
        if bytes == 0 {
            return Vec::new();
        }
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: bytes.next_multiple_of(4),
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = self.device.create_command_encoder(&Default::default());
        encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, bytes.next_multiple_of(4));
        self.queue.submit([encoder.finish()]);
        self.host_read_barrier();
        let slice = staging.slice(..);
        slice.map_async(wgpu::MapMode::Read, |r| r.expect("readback map failed"));
        self.wait();
        let data = slice.get_mapped_range().expect("readback range");
        let out = bytemuck::cast_slice::<u8, T>(&data[..bytes as usize]).to_vec();
        drop(data);
        staging.unmap();
        out
    }
}

impl Gpu {
    /// Make every earlier device write visible to host reads. wgpu records no
    /// host-read barrier before a mapped read, which Vulkan requires; desktop
    /// drivers forgive it, but on Tegra the read returns stale bytes, often a
    /// previous readback's.
    #[cfg(target_os = "linux")]
    fn host_read_barrier(&self) {
        use ash::vk;
        use wgpu::hal::api::Vulkan;
        if self.adapter_info.backend != wgpu::Backend::Vulkan {
            return;
        }
        let Some(raw_device) =
            (unsafe { self.device.as_hal::<Vulkan>() }).map(|d| d.raw_device().clone())
        else {
            return;
        };
        let mut encoder = self.device.create_command_encoder(&Default::default());
        unsafe {
            encoder.as_hal_mut::<Vulkan, _, _>(|hal_encoder| {
                let hal_encoder = hal_encoder.expect("an open Vulkan command encoder");
                let barrier = vk::MemoryBarrier::default()
                    .src_access_mask(vk::AccessFlags::MEMORY_WRITE)
                    .dst_access_mask(vk::AccessFlags::HOST_READ);
                raw_device.cmd_pipeline_barrier(
                    hal_encoder.raw_handle(),
                    vk::PipelineStageFlags::ALL_COMMANDS,
                    vk::PipelineStageFlags::HOST,
                    vk::DependencyFlags::empty(),
                    &[barrier],
                    &[],
                    &[],
                );
            });
        }
        self.queue.submit([encoder.finish()]);
    }

    #[cfg(not(target_os = "linux"))]
    fn host_read_barrier(&self) {}
}

/// Held while compiling shaders. The Tegra Vulkan driver deadlocks when
/// threads build compute pipelines at once against a cold shader cache.
fn compile_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Workgroup counts covering `threads` 1D threads, folded into y past the
/// per-dimension limit. Kernels rebuild the index with `thread_index`.
pub fn groups_1d(threads: u32) -> (u32, u32) {
    let groups = threads.div_ceil(GROUP_SIZE);
    (
        groups.min(MAX_GROUPS_PER_DIM),
        groups.div_ceil(MAX_GROUPS_PER_DIM),
    )
}

/// Per-pass GPU timestamps, in milliseconds, when the device supports them.
pub struct PassTimer {
    query_set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    labels: Vec<&'static str>,
    capacity: u32,
}

impl PassTimer {
    pub fn new(gpu: &Gpu, capacity: u32) -> Option<Self> {
        gpu.timestamp_period?;
        Some(Self {
            query_set: gpu.device.create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("pass timer"),
                ty: wgpu::QueryType::Timestamp,
                count: 2 * capacity,
            }),
            resolve: gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("pass timer resolve"),
                size: 16 * capacity as u64,
                usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }),
            labels: Vec::new(),
            capacity,
        })
    }

    /// Timestamp writes for the next pass, None once the timer is full.
    pub fn next(&mut self, label: &'static str) -> Option<wgpu::ComputePassTimestampWrites<'_>> {
        let i = self.labels.len() as u32;
        if i >= self.capacity {
            return None;
        }
        self.labels.push(label);
        Some(wgpu::ComputePassTimestampWrites {
            query_set: &self.query_set,
            beginning_of_pass_write_index: Some(2 * i),
            end_of_pass_write_index: Some(2 * i + 1),
        })
    }

    pub fn resolve(&self, encoder: &mut wgpu::CommandEncoder) {
        let n = self.labels.len() as u32;
        if n > 0 {
            encoder.resolve_query_set(&self.query_set, 0..2 * n, &self.resolve, 0);
        }
    }

    /// Each timed pass's label and duration, then reset for the next frame.
    pub fn take(&mut self, gpu: &Gpu) -> Vec<(&'static str, f64)> {
        let period = gpu.timestamp_period.unwrap_or(1.0) as f64;
        let ticks: Vec<u64> = gpu.read(&self.resolve, 2 * self.labels.len());
        let out = self
            .labels
            .iter()
            .enumerate()
            .map(|(i, &label)| {
                let span = ticks[2 * i + 1].saturating_sub(ticks[2 * i]);
                (label, span as f64 * period / 1e6)
            })
            .collect();
        self.labels.clear();
        out
    }
}
