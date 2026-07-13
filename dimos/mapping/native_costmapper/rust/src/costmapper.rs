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

use std::fmt;

use dimos_module::native_config;
use lcm_msgs::geometry_msgs::{Point, Pose, Quaternion};
use lcm_msgs::nav_msgs::{MapMetaData, OccupancyGrid};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::Header;
use rayon::prelude::*;
use serde::Deserialize;
use validator::ValidationError;

const PADDING_METERS: f64 = 1.0;
const GAUSSIAN_TRUNCATE: f64 = 4.0;
const SMOOTHED_WEIGHT_MIN: f32 = 0.01;
const UNKNOWN: i8 = -1;
const FREE: i8 = 0;
const OCCUPIED: i8 = 100;

#[native_config]
#[validate(schema(function = "validate_config"))]
pub struct Config {
    pub algo: String,
    pub config: serde_json::Value,
    pub initial_safe_radius_meters: f64,
}

#[derive(Debug)]
pub struct CostmapError(String);

impl CostmapError {
    fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl fmt::Display for CostmapError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for CostmapError {}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct HeightCostConfig {
    resolution: f64,
    frame_id: Option<String>,
    can_pass_under: f64,
    can_climb: f64,
    ignore_noise: f64,
    smoothing: f64,
}

impl Default for HeightCostConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            frame_id: None,
            can_pass_under: 0.6,
            can_climb: 0.15,
            ignore_noise: 0.05,
            smoothing: 1.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct GeneralOccupancyConfig {
    resolution: f64,
    frame_id: Option<String>,
    min_height: f64,
    max_height: f64,
    mark_free_radius: f64,
}

impl Default for GeneralOccupancyConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            frame_id: None,
            min_height: 0.1,
            max_height: 2.0,
            mark_free_radius: 0.4,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct SimpleOccupancyConfig {
    resolution: f64,
    frame_id: Option<String>,
    min_height: f64,
    max_height: f64,
    closing_iterations: i64,
    closing_connectivity: i64,
    can_pass_under: f64,
    can_climb: f64,
    ignore_noise: f64,
    smoothing: f64,
}

impl Default for SimpleOccupancyConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            frame_id: None,
            min_height: 0.1,
            max_height: 2.0,
            closing_iterations: 1,
            closing_connectivity: 2,
            can_pass_under: 0.6,
            can_climb: 0.15,
            ignore_noise: 0.05,
            smoothing: 1.0,
        }
    }
}

enum Algorithm {
    HeightCost(HeightCostConfig),
    General(GeneralOccupancyConfig),
    Simple(SimpleOccupancyConfig),
}

impl Algorithm {
    fn resolution(&self) -> f64 {
        match self {
            Self::HeightCost(config) => config.resolution,
            Self::General(config) => config.resolution,
            Self::Simple(config) => config.resolution,
        }
    }

    fn frame_id<'a>(&'a self, fallback: &'a str) -> &'a str {
        match self {
            Self::HeightCost(config) => config.frame_id.as_deref().unwrap_or(fallback),
            Self::General(config) => config.frame_id.as_deref().unwrap_or(fallback),
            Self::Simple(config) => config.frame_id.as_deref().unwrap_or(fallback),
        }
    }
}

fn validate_config(config: &Config) -> Result<(), ValidationError> {
    parse_algorithm(config).map(|_| ()).map_err(|error| {
        let mut validation_error = ValidationError::new("invalid_costmapper_config");
        validation_error.message = Some(error.to_string().into());
        validation_error
    })?;
    if !config.initial_safe_radius_meters.is_finite() || config.initial_safe_radius_meters < 0.0 {
        return Err(ValidationError::new(
            "initial_safe_radius_meters_must_be_nonnegative",
        ));
    }
    Ok(())
}

fn parse_algorithm(config: &Config) -> Result<Algorithm, CostmapError> {
    let algorithm = match config.algo.as_str() {
        "height_cost" => Algorithm::HeightCost(parse_config(&config.config, "height_cost")?),
        "general" => Algorithm::General(parse_config(&config.config, "general")?),
        "simple" => Algorithm::Simple(parse_config(&config.config, "simple")?),
        name => {
            return Err(CostmapError::new(format!(
                "unknown occupancy algorithm {name:?}"
            )))
        }
    };
    validate_algorithm(&algorithm)?;
    Ok(algorithm)
}

fn parse_config<T>(value: &serde_json::Value, name: &str) -> Result<T, CostmapError>
where
    T: for<'de> Deserialize<'de>,
{
    serde_json::from_value(value.clone())
        .map_err(|error| CostmapError::new(format!("invalid {name} configuration: {error}")))
}

fn validate_algorithm(algorithm: &Algorithm) -> Result<(), CostmapError> {
    let resolution = algorithm.resolution();
    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(CostmapError::new("resolution must be finite and positive"));
    }

    match algorithm {
        Algorithm::HeightCost(config) => {
            if !config.can_pass_under.is_finite() || config.can_pass_under < 0.0 {
                return Err(CostmapError::new(
                    "can_pass_under must be finite and nonnegative",
                ));
            }
            if !config.can_climb.is_finite() || config.can_climb <= 0.0 {
                return Err(CostmapError::new("can_climb must be finite and positive"));
            }
            if !config.ignore_noise.is_finite() || config.ignore_noise < 0.0 {
                return Err(CostmapError::new(
                    "ignore_noise must be finite and nonnegative",
                ));
            }
            if !config.smoothing.is_finite() || config.smoothing < 0.0 {
                return Err(CostmapError::new(
                    "smoothing must be finite and nonnegative",
                ));
            }
        }
        Algorithm::General(config) => {
            validate_height_range(config.min_height, config.max_height)?;
            if !config.mark_free_radius.is_finite() || config.mark_free_radius < 0.0 {
                return Err(CostmapError::new(
                    "mark_free_radius must be finite and nonnegative",
                ));
            }
        }
        Algorithm::Simple(config) => {
            validate_height_range(config.min_height, config.max_height)?;
        }
    }
    Ok(())
}

fn validate_height_range(min_height: f64, max_height: f64) -> Result<(), CostmapError> {
    if !min_height.is_finite() || !max_height.is_finite() {
        return Err(CostmapError::new("height limits must be finite"));
    }
    Ok(())
}

pub fn calculate_costmap(
    message: &PointCloud2,
    config: &Config,
) -> Result<OccupancyGrid, CostmapError> {
    let algorithm = parse_algorithm(config)?;
    let points = extract_xyz(message)?;

    if points.is_empty() {
        return Ok(empty_grid(message, &algorithm));
    }

    match &algorithm {
        Algorithm::HeightCost(config) => height_cost_occupancy(message, &points, config),
        Algorithm::General(config) => general_occupancy(message, &points, config),
        Algorithm::Simple(config) => simple_occupancy(message, &points, config),
    }
}

pub fn apply_initial_safe_radius(grid: &mut OccupancyGrid, radius_meters: f64) {
    if radius_meters <= 0.0 || grid.data.is_empty() {
        return;
    }

    let resolution = f64::from(grid.info.resolution);
    let origin_x = grid.info.origin.position.x;
    let origin_y = grid.info.origin.position.y;
    let effective_radius = radius_meters + resolution * 0.5;
    let radius_squared = effective_radius * effective_radius;
    let width = grid.info.width.max(0) as usize;
    let height = grid.info.height.max(0) as usize;

    for row in 0..height {
        let world_y = row as f64 * resolution + origin_y;
        for column in 0..width {
            let world_x = column as f64 * resolution + origin_x;
            if world_x * world_x + world_y * world_y <= radius_squared {
                grid.data[row * width + column] = FREE;
            }
        }
    }
}

fn height_cost_occupancy(
    message: &PointCloud2,
    points: &[[f64; 3]],
    config: &HeightCostConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_points(points, config.resolution)?;
    let cell_count = bounds.cell_count()?;
    let mut minimum_heights = vec![f32::NAN; cell_count];
    let mut maximum_heights = vec![f32::NAN; cell_count];
    let inverse_resolution = 1.0 / config.resolution;

    for &[x, y, z] in points {
        let column = ((x - bounds.min_x) * inverse_resolution + 0.5) as usize;
        let row = ((y - bounds.min_y) * inverse_resolution + 0.5) as usize;
        if column >= bounds.width || row >= bounds.height {
            continue;
        }
        let index = row * bounds.width + column;
        let z = z as f32;
        if minimum_heights[index].is_nan() || z < minimum_heights[index] {
            minimum_heights[index] = z;
        }
        if maximum_heights[index].is_nan() || z > maximum_heights[index] {
            maximum_heights[index] = z;
        }
    }

    let can_pass_under = config.can_pass_under as f32;
    let mut heights: Vec<f32> = minimum_heights
        .par_iter()
        .zip(&maximum_heights)
        .map(|(minimum, maximum)| {
            if minimum.is_nan() {
                f32::NAN
            } else if maximum - minimum > can_pass_under {
                *minimum
            } else {
                *maximum
            }
        })
        .collect();

    let mut observed: Vec<bool> = heights.iter().map(|height| !height.is_nan()).collect();
    if config.smoothing > 0.0 && observed.iter().any(|value| *value) {
        let weights: Vec<f32> = observed
            .iter()
            .map(|value| u8::from(*value) as f32)
            .collect();
        let filled: Vec<f32> = heights
            .iter()
            .map(|height| if height.is_nan() { 0.0 } else { *height })
            .collect();
        let smoothed_heights =
            gaussian_filter(&filled, bounds.width, bounds.height, config.smoothing);
        let smoothed_weights =
            gaussian_filter(&weights, bounds.width, bounds.height, config.smoothing);
        heights
            .par_iter_mut()
            .zip(&mut observed)
            .zip(smoothed_heights.par_iter().zip(&smoothed_weights))
            .for_each(
                |((height, is_observed), (smoothed_height, smoothed_weight))| {
                    if !*is_observed && *smoothed_weight > SMOOTHED_WEIGHT_MIN {
                        *height = smoothed_height / smoothed_weight;
                        *is_observed = true;
                    }
                },
            );
    }

    let data = if observed.iter().any(|value| *value) {
        height_costs(&heights, &observed, &bounds, config)
    } else {
        vec![UNKNOWN; cell_count]
    };

    Ok(make_grid(
        message,
        &bounds,
        config.frame_id.as_deref(),
        data,
    ))
}

fn height_costs(
    heights: &[f32],
    observed: &[bool],
    bounds: &GridBounds,
    config: &HeightCostConfig,
) -> Vec<i8> {
    let height_for_gradient: Vec<f32> = heights
        .iter()
        .zip(observed)
        .map(|(height, is_observed)| if *is_observed { *height } else { 0.0 })
        .collect();
    let gradient_x = sobel(&height_for_gradient, bounds.width, bounds.height, true);
    let gradient_y = sobel(&height_for_gradient, bounds.width, bounds.height, false);
    let resolution = config.resolution as f32;
    let ignore_noise = config.ignore_noise as f32;
    let can_climb = config.can_climb as f32;
    let mut output = vec![UNKNOWN; heights.len()];

    output
        .par_chunks_mut(bounds.width)
        .enumerate()
        .for_each(|(row, output_row)| {
            if row == 0 || row + 1 >= bounds.height {
                return;
            }
            for (column, output_cell) in output_row
                .iter_mut()
                .enumerate()
                .take(bounds.width.saturating_sub(1))
                .skip(1)
            {
                let index = row * bounds.width + column;
                let valid_gradient = observed[index]
                    && observed[index - 1]
                    && observed[index + 1]
                    && observed[index - bounds.width]
                    && observed[index + bounds.width];
                if !valid_gradient {
                    continue;
                }

                let gradient_magnitude = (gradient_x[index].powi(2) + gradient_y[index].powi(2))
                    .sqrt()
                    / (8.0 * resolution);
                let mut height_change = gradient_magnitude * resolution;
                if height_change < ignore_noise {
                    height_change = 0.0;
                }
                let cost = (height_change / can_climb * 100.0).clamp(0.0, 100.0);
                *output_cell = cost as i8;
            }
        });
    output
}

fn general_occupancy(
    message: &PointCloud2,
    points: &[[f64; 3]],
    config: &GeneralOccupancyConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_points(points, config.resolution)?;
    let mut data = vec![UNKNOWN; bounds.cell_count()?];

    for &[x, y, z] in points {
        if z < config.min_height {
            let index = bounds.clipped_index(x, y, config.resolution);
            data[index] = FREE;
        }
    }
    for &[x, y, z] in points {
        if z >= config.min_height && z <= config.max_height {
            let index = bounds.clipped_index(x, y, config.resolution);
            data[index] = OCCUPIED;
        }
    }

    if config.mark_free_radius > 0.0 {
        dilate_free_space(
            &mut data,
            bounds.width,
            bounds.height,
            (config.mark_free_radius / config.resolution).ceil() as usize,
        );
    }

    Ok(make_grid(
        message,
        &bounds,
        config.frame_id.as_deref(),
        data,
    ))
}

fn simple_occupancy(
    message: &PointCloud2,
    points: &[[f64; 3]],
    config: &SimpleOccupancyConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_points(points, config.resolution)?;
    let mut data = vec![UNKNOWN; bounds.cell_count()?];
    let inverse_resolution = 1.0 / config.resolution;

    for &[x, y, z] in points {
        if z < config.min_height {
            if let Some(index) = bounds.rounded_index(x, y, inverse_resolution) {
                data[index] = FREE;
            }
        }
    }
    for &[x, y, z] in points {
        if z >= config.min_height && z <= config.max_height {
            if let Some(index) = bounds.rounded_index(x, y, inverse_resolution) {
                data[index] = OCCUPIED;
            }
        }
    }

    Ok(make_grid(
        message,
        &bounds,
        config.frame_id.as_deref(),
        data,
    ))
}

#[derive(Debug)]
struct GridBounds {
    min_x: f64,
    min_y: f64,
    width: usize,
    height: usize,
    resolution: f64,
}

impl GridBounds {
    fn from_points(points: &[[f64; 3]], resolution: f64) -> Result<Self, CostmapError> {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for &[x, y, _] in points {
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
        min_x -= PADDING_METERS;
        max_x += PADDING_METERS;
        min_y -= PADDING_METERS;
        max_y += PADDING_METERS;

        let width = ((max_x - min_x) / resolution).ceil();
        let height = ((max_y - min_y) / resolution).ceil();
        if !width.is_finite() || !height.is_finite() || width < 1.0 || height < 1.0 {
            return Err(CostmapError::new(
                "point-cloud bounds produced an invalid grid",
            ));
        }
        if width > i32::MAX as f64 || height > i32::MAX as f64 {
            return Err(CostmapError::new(
                "occupancy grid dimensions exceed i32 limits",
            ));
        }

        Ok(Self {
            min_x,
            min_y,
            width: width as usize,
            height: height as usize,
            resolution,
        })
    }

    fn cell_count(&self) -> Result<usize, CostmapError> {
        self.width
            .checked_mul(self.height)
            .ok_or_else(|| CostmapError::new("occupancy grid allocation overflow"))
    }

    fn clipped_index(&self, x: f64, y: f64, resolution: f64) -> usize {
        let column = ((x - self.min_x) / resolution) as usize;
        let row = ((y - self.min_y) / resolution) as usize;
        row.min(self.height - 1) * self.width + column.min(self.width - 1)
    }

    fn rounded_index(&self, x: f64, y: f64, inverse_resolution: f64) -> Option<usize> {
        let column = ((x - self.min_x) * inverse_resolution + 0.5) as usize;
        let row = ((y - self.min_y) * inverse_resolution + 0.5) as usize;
        (column < self.width && row < self.height).then_some(row * self.width + column)
    }
}

fn empty_grid(message: &PointCloud2, algorithm: &Algorithm) -> OccupancyGrid {
    let bounds = GridBounds {
        min_x: 0.0,
        min_y: 0.0,
        width: 1,
        height: 1,
        resolution: algorithm.resolution(),
    };
    make_grid(
        message,
        &bounds,
        Some(algorithm.frame_id(&message.header.frame_id)),
        vec![UNKNOWN],
    )
}

fn make_grid(
    message: &PointCloud2,
    bounds: &GridBounds,
    frame_id: Option<&str>,
    data: Vec<i8>,
) -> OccupancyGrid {
    let stamp = message.header.stamp.clone();
    OccupancyGrid {
        header: Header {
            seq: 0,
            stamp: stamp.clone(),
            frame_id: frame_id.unwrap_or(&message.header.frame_id).to_string(),
        },
        info: MapMetaData {
            map_load_time: stamp,
            resolution: bounds.resolution as f32,
            width: bounds.width as i32,
            height: bounds.height as i32,
            origin: Pose {
                position: Point {
                    x: bounds.min_x,
                    y: bounds.min_y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        },
        data,
    }
}

fn extract_xyz(message: &PointCloud2) -> Result<Vec<[f64; 3]>, CostmapError> {
    if message.width <= 0 || message.height <= 0 {
        return Ok(Vec::new());
    }
    if message.point_step <= 0 || message.row_step <= 0 {
        return Err(CostmapError::new(
            "point_step and row_step must be positive",
        ));
    }

    let x_offset = field_offset(message, "x")?;
    let y_offset = field_offset(message, "y")?;
    let z_offset = field_offset(message, "z")?;
    let point_step = message.point_step as usize;
    let row_step = message.row_step as usize;
    if [x_offset, y_offset, z_offset].into_iter().any(|offset| {
        offset
            .checked_add(size_of::<f32>())
            .is_none_or(|end| end > point_step)
    }) {
        return Err(CostmapError::new(
            "xyz field offsets do not fit within point_step",
        ));
    }

    let width = message.width as usize;
    let height = message.height as usize;
    let point_count = width
        .checked_mul(height)
        .ok_or_else(|| CostmapError::new("point-cloud dimensions overflow"))?;
    let row_payload = width
        .checked_mul(point_step)
        .ok_or_else(|| CostmapError::new("point-cloud row size overflows"))?;
    if row_step < row_payload {
        return Err(CostmapError::new(
            "row_step is shorter than width times point_step",
        ));
    }
    let required = (height - 1)
        .checked_mul(row_step)
        .and_then(|offset| offset.checked_add(row_payload))
        .ok_or_else(|| CostmapError::new("point-cloud data dimensions overflow"))?;
    if message.data.len() < required {
        return Err(CostmapError::new(
            "data buffer is shorter than the declared cloud dimensions",
        ));
    }

    let mut points = Vec::with_capacity(point_count);
    for row in 0..height {
        for column in 0..width {
            let base = row * row_step + column * point_step;
            let point = [
                read_f32(&message.data, base + x_offset, message.is_bigendian) as f64,
                read_f32(&message.data, base + y_offset, message.is_bigendian) as f64,
                read_f32(&message.data, base + z_offset, message.is_bigendian) as f64,
            ];
            if point.iter().any(|value| !value.is_finite()) {
                return Err(CostmapError::new(
                    "point cloud contains a non-finite xyz coordinate",
                ));
            }
            points.push(point);
        }
    }
    Ok(points)
}

fn field_offset(message: &PointCloud2, name: &str) -> Result<usize, CostmapError> {
    let field = message
        .fields
        .iter()
        .find(|field| field.name == name && field.datatype == PointField::FLOAT32 as u8)
        .ok_or_else(|| CostmapError::new(format!("missing float32 {name} field")))?;
    usize::try_from(field.offset)
        .map_err(|_| CostmapError::new(format!("{name} field offset must be nonnegative")))
}

fn read_f32(data: &[u8], offset: usize, big_endian: bool) -> f32 {
    let bytes: [u8; 4] = data[offset..offset + 4]
        .try_into()
        .expect("field bounds checked before decoding");
    if big_endian {
        f32::from_be_bytes(bytes)
    } else {
        f32::from_le_bytes(bytes)
    }
}

fn gaussian_filter(input: &[f32], width: usize, height: usize, sigma: f64) -> Vec<f32> {
    let radius = (GAUSSIAN_TRUNCATE * sigma + 0.5).floor() as isize;
    let mut kernel = Vec::with_capacity((radius * 2 + 1) as usize);
    let mut sum = 0.0;
    for offset in -radius..=radius {
        let value = (-(offset as f64).powi(2) / (2.0 * sigma * sigma)).exp();
        kernel.push(value);
        sum += value;
    }
    for value in &mut kernel {
        *value /= sum;
    }

    let mut horizontal = vec![0.0; input.len()];
    horizontal
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, output_row)| {
            for (column, output_cell) in output_row.iter_mut().enumerate() {
                let mut value = 0.0;
                for (kernel_index, weight) in kernel.iter().enumerate() {
                    let offset = kernel_index as isize - radius;
                    let source_column = reflect_index(column as isize + offset, width);
                    value += f64::from(input[row * width + source_column]) * weight;
                }
                *output_cell = value as f32;
            }
        });

    let mut output = vec![0.0; input.len()];
    output
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, output_row)| {
            for (column, output_cell) in output_row.iter_mut().enumerate() {
                let mut value = 0.0;
                for (kernel_index, weight) in kernel.iter().enumerate() {
                    let offset = kernel_index as isize - radius;
                    let source_row = reflect_index(row as isize + offset, height);
                    value += f64::from(horizontal[source_row * width + column]) * weight;
                }
                *output_cell = value as f32;
            }
        });
    output
}

fn sobel(input: &[f32], width: usize, height: usize, horizontal: bool) -> Vec<f32> {
    let mut output = vec![0.0; input.len()];
    let smoothing = [1.0_f64, 2.0, 1.0];
    let derivative = [-1.0_f64, 0.0, 1.0];

    output
        .par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, output_row)| {
            for (column, output_cell) in output_row.iter_mut().enumerate() {
                let mut value = 0.0;
                for kernel_row in 0..3 {
                    let source_row = reflect_index(row as isize + kernel_row as isize - 1, height);
                    for kernel_column in 0..3 {
                        let source_column =
                            reflect_index(column as isize + kernel_column as isize - 1, width);
                        let weight = if horizontal {
                            smoothing[kernel_row] * derivative[kernel_column]
                        } else {
                            derivative[kernel_row] * smoothing[kernel_column]
                        };
                        value += f64::from(input[source_row * width + source_column]) * weight;
                    }
                }
                *output_cell = value as f32;
            }
        });
    output
}

fn reflect_index(index: isize, length: usize) -> usize {
    if length <= 1 {
        return 0;
    }
    let period = (length * 2) as isize;
    let reflected = index.rem_euclid(period);
    if reflected >= length as isize {
        (period - 1 - reflected) as usize
    } else {
        reflected as usize
    }
}

fn dilate_free_space(data: &mut [i8], width: usize, height: usize, radius: usize) {
    let free_cells: Vec<usize> = data
        .iter()
        .enumerate()
        .filter_map(|(index, value)| (*value == FREE).then_some(index))
        .collect();
    let mut expanded = vec![false; data.len()];
    let radius = radius as isize;
    let radius_squared = radius * radius;

    for index in free_cells {
        let row = (index / width) as isize;
        let column = (index % width) as isize;
        for row_offset in -radius..=radius {
            for column_offset in -radius..=radius {
                if row_offset * row_offset + column_offset * column_offset > radius_squared {
                    continue;
                }
                let expanded_row = row + row_offset;
                let expanded_column = column + column_offset;
                if expanded_row >= 0
                    && expanded_row < height as isize
                    && expanded_column >= 0
                    && expanded_column < width as isize
                {
                    expanded[expanded_row as usize * width + expanded_column as usize] = true;
                }
            }
        }
    }

    for (value, is_expanded) in data.iter_mut().zip(expanded) {
        if is_expanded && *value != OCCUPIED {
            *value = FREE;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lcm_msgs::std_msgs::Time;

    fn cloud(points: &[[f32; 3]]) -> PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * 12);
        for point in points {
            for coordinate in point {
                data.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        let field = |name: &str, offset: i32| PointField {
            name: name.to_string(),
            offset,
            datatype: PointField::FLOAT32 as u8,
            count: 1,
        };
        PointCloud2 {
            header: Header {
                seq: 7,
                stamp: Time { sec: 12, nsec: 34 },
                frame_id: "world".to_string(),
            },
            height: 1,
            width: points.len() as i32,
            fields: vec![field("x", 0), field("y", 4), field("z", 8)],
            is_bigendian: false,
            point_step: 12,
            row_step: points.len() as i32 * 12,
            data,
            is_dense: true,
        }
    }

    fn config(algo: &str, value: serde_json::Value) -> Config {
        Config {
            algo: algo.to_string(),
            config: value,
            initial_safe_radius_meters: 0.0,
        }
    }

    #[test]
    fn simple_marks_ground_free_and_obstacles_occupied() {
        let message = cloud(&[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.5, 0.0, 3.0]]);
        let result = calculate_costmap(
            &message,
            &config(
                "simple",
                serde_json::json!({"resolution": 0.5, "min_height": 0.1, "max_height": 2.0}),
            ),
        )
        .unwrap();

        assert_eq!(result.info.width, 5);
        assert_eq!(result.info.height, 4);
        assert_eq!(result.data[2 * 5 + 2], OCCUPIED);
        assert_eq!(result.data[2 * 5 + 3], UNKNOWN);
        assert_eq!(result.header.stamp, message.header.stamp);
    }

    #[test]
    fn general_dilates_free_cells_without_overwriting_obstacles() {
        let message = cloud(&[[0.0, 0.0, 0.0], [0.5, 0.0, 0.5]]);
        let result = calculate_costmap(
            &message,
            &config(
                "general",
                serde_json::json!({
                    "resolution": 0.5,
                    "min_height": 0.1,
                    "max_height": 2.0,
                    "mark_free_radius": 0.5
                }),
            ),
        )
        .unwrap();

        let obstacle_index = 2 * result.info.width as usize + 3;
        assert_eq!(result.data[obstacle_index], OCCUPIED);
        assert!(result.data.iter().filter(|value| **value == FREE).count() > 1);
    }

    #[test]
    fn safe_radius_clears_cells_using_grid_world_coordinates() {
        let message = cloud(&[[0.0, 0.0, 0.5]]);
        let mut result = calculate_costmap(
            &message,
            &config(
                "simple",
                serde_json::json!({"resolution": 0.5, "min_height": 0.1, "max_height": 2.0}),
            ),
        )
        .unwrap();

        apply_initial_safe_radius(&mut result, 0.1);

        let center = 2 * result.info.width as usize + 2;
        assert_eq!(result.data[center], FREE);
    }

    #[test]
    fn invalid_algorithm_is_rejected() {
        let error = calculate_costmap(
            &cloud(&[[0.0, 0.0, 0.0]]),
            &config("missing", serde_json::json!({})),
        )
        .unwrap_err();
        assert!(error.to_string().contains("unknown occupancy algorithm"));
    }

    #[test]
    fn invalid_point_field_offset_is_rejected() {
        let mut message = cloud(&[[0.0, 0.0, 0.0]]);
        message.fields[0].offset = -1;

        let error = calculate_costmap(
            &message,
            &config("simple", serde_json::json!({"resolution": 0.5})),
        )
        .unwrap_err();

        assert!(error
            .to_string()
            .contains("field offset must be nonnegative"));
    }

    #[test]
    fn reflect_index_matches_scipy_half_sample_symmetry() {
        let reflected: Vec<usize> = (-5..9).map(|index| reflect_index(index, 4)).collect();
        assert_eq!(reflected, vec![3, 3, 2, 1, 0, 0, 1, 2, 3, 3, 2, 1, 0, 0]);
    }
}
