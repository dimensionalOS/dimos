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
use image::{GrayImage, ImageBuffer, Luma, LumaA};
use imageproc::filter::{filter, separable_filter_equal};
use imageproc::kernel::Kernel;
use imageproc::morphology::{grayscale_dilate, Mask};
use lcm_msgs::geometry_msgs::{Point, Pose, Quaternion};
use lcm_msgs::nav_msgs::{MapMetaData, OccupancyGrid};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::Header;
use serde::{Deserialize, Deserializer};
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

#[derive(Debug, Clone)]
struct FrameId(Option<String>);

impl FrameId {
    fn as_deref(&self) -> Option<&str> {
        self.0.as_deref().filter(|frame_id| !frame_id.is_empty())
    }
}

impl<'de> Deserialize<'de> for FrameId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        match serde_json::Value::deserialize(deserializer)? {
            serde_json::Value::Null => Ok(Self(None)),
            serde_json::Value::String(value) => Ok(Self(Some(value))),
            _ => Err(serde::de::Error::custom(
                "frame_id must be a string or null",
            )),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct HeightCostConfig {
    resolution: f64,
    frame_id: FrameId,
    can_pass_under: f64,
    can_climb: f64,
    ignore_noise: f64,
    smoothing: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct GeneralOccupancyConfig {
    resolution: f64,
    frame_id: FrameId,
    min_height: f64,
    max_height: f64,
    mark_free_radius: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SimpleOccupancyConfig {
    resolution: f64,
    frame_id: FrameId,
    min_height: f64,
    max_height: f64,
    // SimpleOccupancyConfig serializes these fields even though Python's
    // simple_occupancy does not consume them. Deserialize them only to accept
    // the exact same config dictionary at the native drop-in boundary.
    #[serde(rename = "closing_iterations")]
    _closing_iterations: i64,
    #[serde(rename = "closing_connectivity")]
    _closing_connectivity: i64,
    #[serde(rename = "can_pass_under")]
    _can_pass_under: f64,
    #[serde(rename = "can_climb")]
    _can_climb: f64,
    #[serde(rename = "ignore_noise")]
    _ignore_noise: f64,
    #[serde(rename = "smoothing")]
    _smoothing: f64,
}

#[derive(Debug)]
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
    CostmapCalculator::new(config).map(|_| ()).map_err(|error| {
        let mut validation_error = ValidationError::new("invalid_costmapper_config");
        validation_error.message = Some(error.to_string().into());
        validation_error
    })
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
            if (config.mark_free_radius / config.resolution).ceil() > f64::from(u8::MAX) {
                return Err(CostmapError::new(
                    "mark_free_radius must not exceed 255 grid cells",
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
    if min_height > max_height {
        return Err(CostmapError::new(
            "min_height must be less than or equal to max_height",
        ));
    }
    Ok(())
}

#[derive(Debug)]
pub struct CostmapCalculator {
    algorithm: Algorithm,
    initial_safe_radius_meters: f64,
}

impl CostmapCalculator {
    pub fn new(config: &Config) -> Result<Self, CostmapError> {
        if !config.initial_safe_radius_meters.is_finite() || config.initial_safe_radius_meters < 0.0
        {
            return Err(CostmapError::new(
                "initial_safe_radius_meters must be finite and nonnegative",
            ));
        }
        Ok(Self {
            algorithm: parse_algorithm(config)?,
            initial_safe_radius_meters: config.initial_safe_radius_meters,
        })
    }

    pub fn calculate(&self, message: &PointCloud2) -> Result<OccupancyGrid, CostmapError> {
        let cloud = PointCloudView::new(message)?;

        let mut grid = if cloud.is_empty() {
            empty_grid(message, &self.algorithm)
        } else {
            match &self.algorithm {
                Algorithm::HeightCost(config) => height_cost_occupancy(message, &cloud, config)?,
                Algorithm::General(config) => general_occupancy(message, &cloud, config)?,
                Algorithm::Simple(config) => simple_occupancy(message, &cloud, config)?,
            }
        };

        apply_initial_safe_radius(&mut grid, self.initial_safe_radius_meters);
        Ok(grid)
    }
}

fn apply_initial_safe_radius(grid: &mut OccupancyGrid, radius_meters: f64) {
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
    cloud: &PointCloudView<'_>,
    config: &HeightCostConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_cloud(cloud, config.resolution)?;
    let cell_count = bounds.cell_count()?;
    let mut minimum_heights = vec![f32::INFINITY; cell_count];
    let mut maximum_heights = vec![f32::NEG_INFINITY; cell_count];
    let inverse_resolution = 1.0 / config.resolution;
    let min_x = bounds.min_x;
    let min_y = bounds.min_y;
    let width = bounds.width;
    let height = bounds.height;

    cloud.for_each_xyz(|x, y, z| {
        let column = ((x - min_x) * inverse_resolution + 0.5) as usize;
        let row = ((y - min_y) * inverse_resolution + 0.5) as usize;
        if column >= width || row >= height {
            return;
        }
        let index = row * width + column;
        let z = z as f32;
        let min_slot = unsafe { minimum_heights.get_unchecked_mut(index) };
        let max_slot = unsafe { maximum_heights.get_unchecked_mut(index) };
        if z < *min_slot {
            *min_slot = z;
        }
        if z > *max_slot {
            *max_slot = z;
        }
    })?;

    let can_pass_under = config.can_pass_under as f32;
    let mut heights = vec![0.0f32; cell_count];
    let mut observed = vec![false; cell_count];
    for index in 0..cell_count {
        let minimum = minimum_heights[index];
        if minimum.is_finite() {
            let maximum = maximum_heights[index];
            heights[index] = if maximum - minimum > can_pass_under {
                minimum
            } else {
                maximum
            };
            observed[index] = true;
        }
    }

    if config.smoothing > 0.0 && observed.iter().any(|value| *value) {
        let mut weights = vec![0.0f32; cell_count];
        for (weight, is_observed) in weights.iter_mut().zip(&observed) {
            if *is_observed {
                *weight = 1.0;
            }
        }
        let (smoothed_heights, smoothed_weights) =
            gaussian_filter_pair(&heights, &weights, width, height, config.smoothing)?;
        for index in 0..cell_count {
            if !observed[index] && smoothed_weights[index] > SMOOTHED_WEIGHT_MIN {
                heights[index] = smoothed_heights[index] / smoothed_weights[index];
                observed[index] = true;
            }
        }
    }

    let data = if observed.iter().any(|value| *value) {
        height_costs(&heights, &observed, &bounds, config)?
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
) -> Result<Vec<i8>, CostmapError> {
    let (gradient_x, gradient_y) = sobel_pair(heights, bounds.width, bounds.height)?;
    let resolution = config.resolution as f32;
    let ignore_noise = config.ignore_noise as f32;
    let can_climb = config.can_climb as f32;
    let scale = 1.0 / (8.0 * resolution);
    let mut output = vec![UNKNOWN; heights.len()];

    for row in 1..bounds.height.saturating_sub(1) {
        let row_offset = row * bounds.width;
        for column in 1..bounds.width.saturating_sub(1) {
            let index = row_offset + column;
            let valid_gradient = observed[index]
                && observed[index - 1]
                && observed[index + 1]
                && observed[index - bounds.width]
                && observed[index + bounds.width];
            if !valid_gradient {
                continue;
            }

            let gradient_magnitude = (gradient_x[index] * gradient_x[index]
                + gradient_y[index] * gradient_y[index])
                .sqrt()
                * scale;
            let mut height_change = gradient_magnitude * resolution;
            if height_change < ignore_noise {
                height_change = 0.0;
            }
            let cost = (height_change / can_climb * 100.0).clamp(0.0, 100.0);
            output[index] = cost as i8;
        }
    }
    Ok(output)
}

fn general_occupancy(
    message: &PointCloud2,
    cloud: &PointCloudView<'_>,
    config: &GeneralOccupancyConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_cloud(cloud, config.resolution)?;
    let mut data = vec![UNKNOWN; bounds.cell_count()?];
    let resolution = config.resolution;
    let min_height = config.min_height;
    let max_height = config.max_height;

    cloud.for_each_xyz(|x, y, z| {
        if z < min_height {
            let index = bounds.clipped_index(x, y, resolution);
            data[index] = FREE;
        }
    })?;
    cloud.for_each_xyz(|x, y, z| {
        if z >= min_height && z <= max_height {
            let index = bounds.clipped_index(x, y, resolution);
            data[index] = OCCUPIED;
        }
    })?;

    if config.mark_free_radius > 0.0 {
        dilate_free_space(
            &mut data,
            bounds.width,
            bounds.height,
            (config.mark_free_radius / config.resolution).ceil() as usize,
        )?;
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
    cloud: &PointCloudView<'_>,
    config: &SimpleOccupancyConfig,
) -> Result<OccupancyGrid, CostmapError> {
    let bounds = GridBounds::from_cloud(cloud, config.resolution)?;
    let mut data = vec![UNKNOWN; bounds.cell_count()?];
    let inverse_resolution = 1.0 / config.resolution;
    let min_height = config.min_height;
    let max_height = config.max_height;

    cloud.for_each_xyz(|x, y, z| {
        if z < min_height {
            if let Some(index) = bounds.rounded_index(x, y, inverse_resolution) {
                data[index] = FREE;
            }
        }
    })?;
    cloud.for_each_xyz(|x, y, z| {
        if z >= min_height && z <= max_height {
            if let Some(index) = bounds.rounded_index(x, y, inverse_resolution) {
                data[index] = OCCUPIED;
            }
        }
    })?;

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
    fn from_cloud(cloud: &PointCloudView<'_>, resolution: f64) -> Result<Self, CostmapError> {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        cloud.for_each_xyz(|x, y, _| {
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        })?;
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

struct PointCloudView<'a> {
    data: &'a [u8],
    width: usize,
    height: usize,
    point_step: usize,
    row_step: usize,
    x_offset: usize,
    y_offset: usize,
    z_offset: usize,
    big_endian: bool,
}

impl<'a> PointCloudView<'a> {
    fn new(message: &'a PointCloud2) -> Result<Self, CostmapError> {
        if message.width <= 0 || message.height <= 0 {
            return Ok(Self {
                data: &[],
                width: 0,
                height: 0,
                point_step: 0,
                row_step: 0,
                x_offset: 0,
                y_offset: 0,
                z_offset: 0,
                big_endian: false,
            });
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

        Ok(Self {
            data: &message.data[..required],
            width,
            height,
            point_step,
            row_step,
            x_offset,
            y_offset,
            z_offset,
            big_endian: message.is_bigendian,
        })
    }

    fn is_empty(&self) -> bool {
        self.width == 0 || self.height == 0
    }

    fn for_each_xyz(&self, mut visit: impl FnMut(f64, f64, f64)) -> Result<(), CostmapError> {
        if self.is_empty() {
            return Ok(());
        }

        // Dense XYZ little-endian clouds are the common DimOS path (point_step 12 or 16).
        if !self.big_endian
            && self.x_offset == 0
            && self.y_offset == 4
            && self.z_offset == 8
            && self.point_step >= 12
        {
            for row in 0..self.height {
                let row_base = row * self.row_step;
                for column in 0..self.width {
                    let base = row_base + column * self.point_step;
                    let x =
                        f32::from_le_bytes(self.data[base..base + 4].try_into().unwrap()) as f64;
                    let y = f32::from_le_bytes(self.data[base + 4..base + 8].try_into().unwrap())
                        as f64;
                    let z = f32::from_le_bytes(self.data[base + 8..base + 12].try_into().unwrap())
                        as f64;
                    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
                        return Err(CostmapError::new(
                            "point cloud contains a non-finite xyz coordinate",
                        ));
                    }
                    visit(x, y, z);
                }
            }
            return Ok(());
        }

        for row in 0..self.height {
            for column in 0..self.width {
                let base = row * self.row_step + column * self.point_step;
                let x = read_f32(self.data, base + self.x_offset, self.big_endian) as f64;
                let y = read_f32(self.data, base + self.y_offset, self.big_endian) as f64;
                let z = read_f32(self.data, base + self.z_offset, self.big_endian) as f64;
                if !(x.is_finite() && y.is_finite() && z.is_finite()) {
                    return Err(CostmapError::new(
                        "point cloud contains a non-finite xyz coordinate",
                    ));
                }
                visit(x, y, z);
            }
        }
        Ok(())
    }
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

fn gaussian_kernel(sigma: f64) -> Result<(Vec<f32>, usize), CostmapError> {
    let radius = (GAUSSIAN_TRUNCATE * sigma + 0.5).floor() as usize;
    let kernel_size = radius
        .checked_mul(2)
        .and_then(|value| value.checked_add(1))
        .ok_or_else(|| CostmapError::new("Gaussian kernel size overflow"))?;
    let mut kernel = Vec::with_capacity(kernel_size);
    let mut sum = 0.0f64;
    for offset in -(radius as isize)..=(radius as isize) {
        let value = (-(offset as f64).powi(2) / (2.0 * sigma * sigma)).exp();
        kernel.push(value as f32);
        sum += value;
    }
    debug_assert_eq!(kernel.len(), kernel_size);
    let inv_sum = (1.0 / sum) as f32;
    for value in &mut kernel {
        *value *= inv_sum;
    }
    Ok((kernel, radius))
}

fn gaussian_filter_pair(
    input_a: &[f32],
    input_b: &[f32],
    width: usize,
    height: usize,
    sigma: f64,
) -> Result<(Vec<f32>, Vec<f32>), CostmapError> {
    if width == 0 || height == 0 {
        return Ok((Vec::new(), Vec::new()));
    }
    let (kernel, radius) = gaussian_kernel(sigma)?;
    let padding = radius
        .checked_mul(2)
        .ok_or_else(|| CostmapError::new("Gaussian padding overflow"))?;
    let padded_width = width
        .checked_add(padding)
        .ok_or_else(|| CostmapError::new("Gaussian image width overflow"))?;
    let padded_height = height
        .checked_add(padding)
        .ok_or_else(|| CostmapError::new("Gaussian image height overflow"))?;
    let channel_count = padded_width
        .checked_mul(padded_height)
        .and_then(|value| value.checked_mul(2))
        .ok_or_else(|| CostmapError::new("Gaussian image allocation overflow"))?;
    let padded_width_u32 = u32::try_from(padded_width)
        .map_err(|_| CostmapError::new("Gaussian image width exceeds u32 limits"))?;
    let padded_height_u32 = u32::try_from(padded_height)
        .map_err(|_| CostmapError::new("Gaussian image height exceeds u32 limits"))?;

    // imageproc pads by continuity. Reflect-pad explicitly before filtering so
    // the cropped result retains scipy.ndimage's half-sample symmetry.
    let radius = radius as isize;
    let mut pixels = Vec::with_capacity(channel_count);
    for padded_row in 0..padded_height {
        let row = reflect_index(padded_row as isize - radius, height);
        for padded_column in 0..padded_width {
            let column = reflect_index(padded_column as isize - radius, width);
            let index = row * width + column;
            pixels.push(input_a[index]);
            pixels.push(input_b[index]);
        }
    }
    let image =
        ImageBuffer::<LumaA<f32>, Vec<f32>>::from_raw(padded_width_u32, padded_height_u32, pixels)
            .ok_or_else(|| CostmapError::new("failed to construct Gaussian image"))?;
    let filtered = separable_filter_equal(&image, &kernel);
    let filtered = filtered.as_raw();

    let mut output_a = Vec::with_capacity(input_a.len());
    let mut output_b = Vec::with_capacity(input_b.len());
    for row in 0..height {
        for column in 0..width {
            let index = ((row + radius as usize) * padded_width + column + radius as usize) * 2;
            output_a.push(filtered[index]);
            output_b.push(filtered[index + 1]);
        }
    }
    Ok((output_a, output_b))
}

fn sobel_pair(
    input: &[f32],
    width: usize,
    height: usize,
) -> Result<(Vec<f32>, Vec<f32>), CostmapError> {
    if width < 3 || height < 3 {
        return Ok((vec![0.0; input.len()], vec![0.0; input.len()]));
    }
    let width = u32::try_from(width)
        .map_err(|_| CostmapError::new("Sobel image width exceeds u32 limits"))?;
    let height = u32::try_from(height)
        .map_err(|_| CostmapError::new("Sobel image height exceeds u32 limits"))?;
    let image = ImageBuffer::<Luma<f32>, Vec<f32>>::from_raw(width, height, input.to_vec())
        .ok_or_else(|| CostmapError::new("failed to construct Sobel image"))?;
    let horizontal = Kernel::new(&[-1.0f32, 0.0, 1.0, -2.0, 0.0, 2.0, -1.0, 0.0, 1.0], 3, 3);
    let vertical = Kernel::new(&[-1.0f32, -2.0, -1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0], 3, 3);
    let gradient_x: ImageBuffer<Luma<f32>, Vec<f32>> = filter(&image, horizontal, |value| value);
    let gradient_y: ImageBuffer<Luma<f32>, Vec<f32>> = filter(&image, vertical, |value| value);
    Ok((gradient_x.into_raw(), gradient_y.into_raw()))
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

fn dilate_free_space(
    data: &mut [i8],
    width: usize,
    height: usize,
    radius: usize,
) -> Result<(), CostmapError> {
    let width = u32::try_from(width)
        .map_err(|_| CostmapError::new("morphology image width exceeds u32 limits"))?;
    let height = u32::try_from(height)
        .map_err(|_| CostmapError::new("morphology image height exceeds u32 limits"))?;
    let radius = u8::try_from(radius)
        .map_err(|_| CostmapError::new("free-space dilation radius exceeds 255 cells"))?;
    let mask = GrayImage::from_raw(
        width,
        height,
        data.iter()
            .map(|value| if *value == FREE { u8::MAX } else { 0 })
            .collect(),
    )
    .ok_or_else(|| CostmapError::new("failed to construct morphology image"))?;
    let expanded = grayscale_dilate(&mask, &Mask::disk(radius));

    for (value, is_expanded) in data.iter_mut().zip(expanded.into_raw()) {
        if is_expanded != 0 && *value != OCCUPIED {
            *value = FREE;
        }
    }
    Ok(())
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

    fn simple_config(resolution: f64) -> Config {
        config(
            "simple",
            serde_json::json!({
                "resolution": resolution,
                "frame_id": null,
                "min_height": 0.1,
                "max_height": 2.0,
                "closing_iterations": 1,
                "closing_connectivity": 2,
                "can_pass_under": 0.6,
                "can_climb": 0.15,
                "ignore_noise": 0.05,
                "smoothing": 1.0
            }),
        )
    }

    fn calculate(message: &PointCloud2, config: &Config) -> Result<OccupancyGrid, CostmapError> {
        CostmapCalculator::new(config)?.calculate(message)
    }

    #[test]
    fn simple_marks_ground_free_and_obstacles_occupied() {
        let message = cloud(&[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.5, 0.0, 3.0]]);
        let result = calculate(&message, &simple_config(0.5)).unwrap();

        assert_eq!(result.info.width, 5);
        assert_eq!(result.info.height, 4);
        assert_eq!(result.data[2 * 5 + 2], OCCUPIED);
        assert_eq!(result.data[2 * 5 + 3], UNKNOWN);
        assert_eq!(result.header.stamp, message.header.stamp);
    }

    #[test]
    fn empty_configured_frame_id_uses_cloud_frame() {
        let message = cloud(&[[0.0, 0.0, 0.0]]);
        let mut config = simple_config(0.5);
        config.config["frame_id"] = serde_json::Value::String(String::new());

        let result = calculate(&message, &config).unwrap();

        assert_eq!(result.header.frame_id, message.header.frame_id);
    }

    #[test]
    fn general_dilates_free_cells_without_overwriting_obstacles() {
        let message = cloud(&[[0.0, 0.0, 0.0], [0.5, 0.0, 0.5]]);
        let result = calculate(
            &message,
            &config(
                "general",
                serde_json::json!({
                    "resolution": 0.5,
                    "frame_id": null,
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
        let mut config = simple_config(0.5);
        config.initial_safe_radius_meters = 0.1;
        let result = calculate(&message, &config).unwrap();

        let center = 2 * result.info.width as usize + 2;
        assert_eq!(result.data[center], FREE);
    }

    #[test]
    fn invalid_algorithm_is_rejected() {
        let error = CostmapCalculator::new(&config("missing", serde_json::json!({}))).unwrap_err();
        assert!(error.to_string().contains("unknown occupancy algorithm"));
    }

    #[test]
    fn dilation_radius_larger_than_imageproc_supports_is_rejected() {
        let error = CostmapCalculator::new(&config(
            "general",
            serde_json::json!({
                "resolution": 0.01,
                "frame_id": null,
                "min_height": 0.1,
                "max_height": 2.0,
                "mark_free_radius": 2.56
            }),
        ))
        .unwrap_err();

        assert!(error
            .to_string()
            .contains("mark_free_radius must not exceed 255 grid cells"));
    }

    #[test]
    fn missing_python_owned_config_field_is_rejected() {
        let error = CostmapCalculator::new(&config(
            "simple",
            serde_json::json!({
                "resolution": 0.5,
                "min_height": 0.1,
                "max_height": 2.0
            }),
        ))
        .unwrap_err();

        assert!(error.to_string().contains("missing field `frame_id`"));
    }

    #[test]
    fn invalid_point_field_offset_is_rejected() {
        let mut message = cloud(&[[0.0, 0.0, 0.0]]);
        message.fields[0].offset = -1;

        let error = calculate(&message, &simple_config(0.5)).unwrap_err();

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
