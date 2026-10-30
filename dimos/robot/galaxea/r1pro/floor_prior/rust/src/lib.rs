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

//! The R1 Pro's head depth: `depth2depth_cloud` with a floor prior. The chassis lidar never sees the floor near the
//! robot, so unanchored the near field reads 2-5x short; each frame, pixels the calibration puts on or just over
//! `height_frame`'s floor (its z = 0) are anchored on it, in rounds, before the frame is calibrated.

use std::sync::Arc;

use dimos_depth2depth_cloud::depth2depth::{Anchor, Calibration};
use dimos_depth2depth_cloud::module::{Config as DepthConfig, FrameHooks, Pipeline};
use dimos_depth2depth_cloud::undistort::UndistortMap;
use dimos_module::{Builder, Input, Module, NativeConfig};
use lcm_msgs::sensor_msgs::{CameraInfo, CompressedImage, PointCloud2};
use nalgebra::{Isometry3, Point3, Vector3};
use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize};
use validator::Validate;

/// Depth2DepthCloud's config plus the cameras and the floor prior's, one flat object as the Python wrapper sends it.
#[derive(Debug, Serialize, Validate)]
pub struct Config {
    /// Which head cameras get depth; each one runs its own model.
    pub head_left: bool,
    pub head_right: bool,
    /// Pixels probed one per block of this many.
    #[validate(range(min = 1, max = 256))]
    pub floor_anchor_step: i64,
    /// A probed pixel the calibration puts at most this above the floor, and no deeper under it than
    /// Depth2DepthCloud's `min_height_m` (deeper is a drop, left as measured), is anchored on it.
    #[validate(range(min = 0.0, max = 10.0))]
    pub floor_tolerance_m: f64,
    /// Probe-and-anchor rounds per frame: each round's anchors recalibrate the pixels the next one probes.
    #[validate(range(min = 1, max = 16))]
    pub floor_anchor_rounds: i64,
    /// A cloud point no higher than this above the floor (and no deeper than `min_height_m`) is slid along its ray onto it.
    #[validate(range(min = 0.0, max = 10.0))]
    pub floor_snap_tolerance_m: f64,
    /// The floor under the robot, which it stands on and no sensor sees, as points this far apart; 0 radius adds none.
    #[validate(range(min = 0.0, max = 10.0))]
    pub footprint_radius_m: f64,
    #[validate(range(min = 0.001, max = 10.0))]
    pub footprint_spacing_m: f64,
    /// Unseen floor past the footprint, per bearing out to its nearest point within this reach if that is floor; 0 adds none.
    #[validate(range(min = 0.0, max = 100.0))]
    pub view_fill_reach_m: f64,
    #[validate(range(min = 0.1, max = 180.0))]
    pub view_fill_bin_deg: f64,
    #[serde(flatten)]
    #[validate(nested)]
    pub depth: DepthConfig,
    /// Depth2DepthCloud's `max_range_m` and `min_height_m`: the floor is counted out to the first, drops start under the second.
    #[serde(skip)]
    max_range_m: f64,
    #[serde(skip)]
    min_height_m: f64,
}

impl NativeConfig for Config {}

fn field<T: serde::de::DeserializeOwned, E: serde::de::Error>(
    value: serde_json::Value,
) -> Result<T, E> {
    serde_json::from_value(value).map_err(E::custom)
}

impl<'de> Deserialize<'de> for Config {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let mut fields = serde_json::Map::deserialize(deserializer)?;
        let mut take = |name: &'static str| {
            fields
                .remove(name)
                .ok_or_else(|| D::Error::missing_field(name))
        };
        let head_left = take("head_left")?;
        let head_right = take("head_right")?;
        let floor_anchor_step = take("floor_anchor_step")?;
        let floor_tolerance_m = take("floor_tolerance_m")?;
        let floor_anchor_rounds = take("floor_anchor_rounds")?;
        let floor_snap_tolerance_m = take("floor_snap_tolerance_m")?;
        let footprint_radius_m = take("footprint_radius_m")?;
        let footprint_spacing_m = take("footprint_spacing_m")?;
        let view_fill_reach_m = take("view_fill_reach_m")?;
        let view_fill_bin_deg = take("view_fill_bin_deg")?;
        let depth_field = |name: &'static str| {
            fields
                .get(name)
                .and_then(serde_json::Value::as_f64)
                .ok_or_else(|| D::Error::missing_field(name))
        };
        let max_range_m = depth_field("max_range_m")?;
        let min_height_m = depth_field("min_height_m")?;
        Ok(Config {
            head_left: field::<_, D::Error>(head_left)?,
            head_right: field::<_, D::Error>(head_right)?,
            floor_anchor_step: field::<_, D::Error>(floor_anchor_step)?,
            floor_tolerance_m: field::<_, D::Error>(floor_tolerance_m)?,
            floor_anchor_rounds: field::<_, D::Error>(floor_anchor_rounds)?,
            floor_snap_tolerance_m: field::<_, D::Error>(floor_snap_tolerance_m)?,
            footprint_radius_m: field::<_, D::Error>(footprint_radius_m)?,
            footprint_spacing_m: field::<_, D::Error>(footprint_spacing_m)?,
            view_fill_reach_m: field::<_, D::Error>(view_fill_reach_m)?,
            view_fill_bin_deg: field::<_, D::Error>(view_fill_bin_deg)?,
            depth: field::<_, D::Error>(serde_json::Value::Object(fields))?,
            max_range_m,
            min_height_m,
        })
    }
}

/// A Depth2DepthCloud pipeline per enabled head camera, each with FloorPrior adding to its frames' anchors,
/// all publishing on `head_depth_raw` (each cloud in its own camera's frame).
pub struct R1ProHeadDepth {
    head_left_color: Input<CompressedImage>,
    head_left_info: Input<CameraInfo>,
    head_right_color: Input<CompressedImage>,
    head_right_info: Input<CameraInfo>,
    lidar: Input<PointCloud2>,
    left: Option<Pipeline>,
    right: Option<Pipeline>,
}

impl Module for R1ProHeadDepth {
    type Config = Config;

    fn build(builder: &mut Builder, config: Config) -> Self {
        let floor: Arc<dyn FrameHooks> = Arc::new(FloorPrior {
            step: config.floor_anchor_step as usize,
            tolerance: config.floor_tolerance_m,
            min_height: config.min_height_m,
            rounds: config.floor_anchor_rounds as usize,
            max_range: config.max_range_m,
            snap: FloorSnap {
                tolerance: config.floor_snap_tolerance_m,
                min_height: config.min_height_m,
                footprint_radius: config.footprint_radius_m,
                footprint_spacing: config.footprint_spacing_m,
                view_fill_reach: config.view_fill_reach_m,
                view_fill_bin_deg: config.view_fill_bin_deg,
            },
        });
        let tf = builder.tf();
        let output = builder.output("head_cloud", PointCloud2::encode);
        let pipeline = |enabled: bool| {
            enabled.then(|| {
                Pipeline::new(config.depth.clone(), tf.clone(), output.clone())
                    .with_frame_hooks(floor.clone())
            })
        };
        Self {
            head_left_color: builder.input("head_left_color", CompressedImage::decode),
            head_left_info: builder.input("head_left_info", CameraInfo::decode),
            head_right_color: builder.input("head_right_color", CompressedImage::decode),
            head_right_info: builder.input("head_right_info", CameraInfo::decode),
            lidar: builder.input("lidar", PointCloud2::decode),
            left: pipeline(config.head_left),
            right: pipeline(config.head_right),
        }
    }

    async fn setup(&mut self) {
        for pipeline in self.left.iter_mut().chain(self.right.iter_mut()) {
            pipeline.start();
        }
    }

    async fn handle(&mut self) {
        loop {
            tokio::select! {
                Some(msg) = self.head_left_color.recv() => {
                    if let Some(left) = &self.left { left.on_image(msg) }
                }
                Some(msg) = self.head_left_info.recv() => {
                    if let Some(left) = &self.left { left.on_camera_info(msg) }
                }
                Some(msg) = self.head_right_color.recv() => {
                    if let Some(right) = &self.right { right.on_image(msg) }
                }
                Some(msg) = self.head_right_info.recv() => {
                    if let Some(right) = &self.right { right.on_camera_info(msg) }
                }
                Some(msg) = self.lidar.recv() => {
                    for pipeline in self.left.iter().chain(self.right.iter()) {
                        pipeline.on_lidar(msg.clone());
                    }
                }
                else => break,
            }
        }
    }

    async fn teardown(&mut self) {
        for pipeline in self.left.iter_mut().chain(self.right.iter_mut()) {
            pipeline.stop().await;
        }
    }
}

impl FrameHooks for FloorPrior {
    fn anchors(
        &self,
        calibration: &Calibration,
        pred: &[f32],
        map: &UndistortMap,
        visible: &[Anchor],
        height_from_camera: Option<&Isometry3<f64>>,
    ) -> Vec<Anchor> {
        height_from_camera.map_or_else(Vec::new, |floor_from_camera| {
            self.floor_anchors(calibration, pred, map, visible, floor_from_camera)
        })
    }

    fn points(
        &self,
        points: Vec<[f32; 3]>,
        height_from_camera: Option<&Isometry3<f64>>,
    ) -> Vec<[f32; 3]> {
        match height_from_camera {
            Some(floor_from_camera) => self.snap.apply(points, floor_from_camera),
            None => points,
        }
    }
}

/// Floor prior: the Config fields of the same names (`height_frame`'s z = 0 is the floor), and the floor counted out to `max_range` along the optical axis.
struct FloorPrior {
    step: usize,
    tolerance: f64,
    min_height: f64,
    rounds: usize,
    max_range: f64,
    snap: FloorSnap,
}

/// The finished cloud's floor: the Config fields `floor_snap_tolerance_m`, `min_height_m`, `footprint_*` and `view_fill_*`.
struct FloorSnap {
    tolerance: f64,
    min_height: f64,
    footprint_radius: f64,
    footprint_spacing: f64,
    view_fill_reach: f64,
    view_fill_bin_deg: f64,
}

impl FloorSnap {
    /// Camera-frame points near the floor slid along their rays onto it, then the unseen floor ahead and under the robot added.
    fn apply(&self, points: Vec<[f32; 3]>, floor_from_camera: &Isometry3<f64>) -> Vec<[f32; 3]> {
        let camera_z = floor_from_camera.translation.vector.z;
        let mut in_floor: Vec<Point3<f64>> = Vec::with_capacity(points.len());
        let mut out: Vec<[f32; 3]> = Vec::with_capacity(points.len());
        for [x, y, z] in points {
            let mut point = Point3::new(x as f64, y as f64, z as f64);
            let mut floor_point = floor_from_camera * point;
            let height = floor_point.z;
            if height < camera_z - 1e-3 && (self.min_height..=self.tolerance).contains(&height) {
                point.coords *= -camera_z / (height - camera_z);
                floor_point = floor_from_camera * point;
            }
            in_floor.push(floor_point);
            out.push([point.x as f32, point.y as f32, point.z as f32]);
        }
        let camera_from_floor = floor_from_camera.inverse();
        let mut added: Vec<[f64; 2]> = Vec::new();
        if self.view_fill_reach > 0.0 {
            added.extend(self.view_fill(&in_floor));
        }
        if self.footprint_radius > 0.0 {
            added.extend(self.footprint());
        }
        out.extend(added.into_iter().map(|[x, y]| {
            let point = camera_from_floor * Point3::new(x, y, 0.0);
            [point.x as f32, point.y as f32, point.z as f32]
        }));
        out
    }

    /// Floor (x, y) on a `footprint_spacing` grid over the disc of `footprint_radius` around the floor frame's origin.
    fn footprint(&self) -> Vec<[f64; 2]> {
        let radius = self.footprint_radius;
        let grid = arange(-radius, radius + 1e-9, self.footprint_spacing);
        grid.iter()
            .flat_map(|&y| grid.iter().map(move |&x| [x, y]))
            .filter(|&[x, y]| x.hypot(y) <= radius)
            .collect()
    }

    /// Floor (x, y) from the footprint out to the nearest point per bearing within `view_fill_reach`, where that point is floor.
    fn view_fill(&self, in_floor: &[Point3<f64>]) -> Vec<[f64; 2]> {
        let (inner, reach, bin_deg) = (
            self.footprint_radius,
            self.view_fill_reach,
            self.view_fill_bin_deg,
        );
        let bins = (360.0 / bin_deg).ceil() as usize;
        let bin =
            |x: f64, y: f64| (y.atan2(x).to_degrees().rem_euclid(360.0) / bin_deg).floor() as usize;
        // Per bearing: the radius of its nearest point, and whether that point is floor.
        let mut nearest: Vec<Option<(f64, bool)>> = vec![None; bins];
        for point in in_floor {
            let radius = point.x.hypot(point.y);
            if radius > inner && radius < reach {
                let slot = &mut nearest[bin(point.x, point.y)];
                if slot.is_none_or(|(best, _)| radius < best) {
                    *slot = Some((radius, point.z.abs() < 1e-3));
                }
            }
        }
        let limit: Vec<f64> = nearest
            .iter()
            .map(|slot| match slot {
                Some((radius, true)) => *radius,
                _ => 0.0,
            })
            .collect();
        let grid = arange(-reach, reach + 1e-9, self.footprint_spacing);
        grid.iter()
            .flat_map(|&y| grid.iter().map(move |&x| [x, y]))
            .filter(|&[x, y]| {
                let radius = x.hypot(y);
                radius > inner && radius < limit[bin(x, y)]
            })
            .collect()
    }
}

/// numpy's `arange(start, stop, step)`: `start + i * step` while below `stop`.
fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let count = ((stop - start) / step).ceil().max(0.0) as usize;
    (0..count).map(|i| start + i as f64 * step).collect()
}

impl FloorPrior {
    /// Floor anchors (`floor_from_camera`'s z = 0) for the probed pixels the calibration puts on, just over or just under the floor.
    fn floor_anchors(
        &self,
        calibration: &Calibration,
        pred: &[f32],
        map: &UndistortMap,
        visible: &[Anchor],
        floor_from_camera: &Isometry3<f64>,
    ) -> Vec<Anchor> {
        let pin = &map.camera;
        let camera_z = floor_from_camera.translation.vector.z;
        let candidates: Vec<(usize, usize, f64, f64)> = (0..map.height)
            .step_by(self.step)
            .flat_map(|row| {
                (0..map.width)
                    .step_by(self.step)
                    .map(move |column| (column, row))
            })
            .filter_map(|(column, row)| {
                let ray = Vector3::new(
                    (column as f64 - pin.cx as f64) / pin.fx as f64,
                    (row as f64 - pin.cy as f64) / pin.fy as f64,
                    1.0,
                );
                // Height gained per unit of depth along the optical axis; the floor is only below the camera.
                let down = (floor_from_camera.rotation * ray).z;
                let depth = -camera_z / down;
                (down < 0.0 && depth <= self.max_range).then_some((column, row, depth, down))
            })
            .collect();
        let pixels: Vec<(usize, usize)> = candidates
            .iter()
            .map(|&(column, row, _, _)| (column, row))
            .collect();
        let mut anchored = vec![false; candidates.len()];
        let mut floor: Vec<Anchor> = Vec::new();
        // Floor read too near drops onto the floor once its neighbours are anchored; an obstacle keeps its height.
        for _ in 0..self.rounds {
            let with_floor: Vec<Anchor> = visible.iter().chain(&floor).copied().collect();
            let probed = calibration.probe(pred, map.height, map.width, &with_floor, &pixels);
            let before = floor.len();
            for (index, (&(column, row, depth, down), probed)) in
                candidates.iter().zip(probed).enumerate()
            {
                let height = camera_z + probed as f64 * down;
                if !anchored[index] && (self.min_height..=self.tolerance).contains(&height) {
                    anchored[index] = true;
                    floor.push(Anchor {
                        u: column as f32,
                        v: row as f32,
                        z: depth as f32,
                    });
                }
            }
            if floor.len() == before {
                break;
            }
        }
        floor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_depth2depth_cloud::depth2depth::CalibrationConfig;
    use dimos_depth2depth_cloud::undistort::{Lens, UndistortMap};
    use lcm_msgs::sensor_msgs::CameraInfo;
    use nalgebra::Point3;

    /// A camera 1 m up, pitched 30 degrees down: its 160x120 pinhole map and its pose in the floor frame.
    fn camera() -> (UndistortMap, Isometry3<f64>) {
        // Optical axes (x right, y down, z ahead) to base (x ahead, z up).
        let optical = nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
            0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0,
        ));
        let pitch = nalgebra::Rotation3::from_axis_angle(&Vector3::y_axis(), 30f64.to_radians());
        let floor_from_camera = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 0.0, 1.0),
            nalgebra::UnitQuaternion::from_rotation_matrix(&(pitch * optical)),
        );
        let lens = Lens::from_info(
            &CameraInfo {
                width: 160,
                height: 120,
                K: [100.0, 0.0, 79.5, 0.0, 100.0, 59.5, 0.0, 0.0, 1.0],
                D: vec![0.0; 5],
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        (UndistortMap::new(&lens, 160, 120, 100.0), floor_from_camera)
    }

    /// Depth along the optical axis to the floor for a pixel, and whether that floor point is in the 1.5-1.8 m patch ahead.
    fn floor_hit(
        map: &UndistortMap,
        floor_from_camera: &Isometry3<f64>,
        column: usize,
        row: usize,
    ) -> Option<(f64, bool)> {
        let pin = &map.camera;
        let ray = Vector3::new(
            (column as f64 - pin.cx as f64) / pin.fx as f64,
            (row as f64 - pin.cy as f64) / pin.fy as f64,
            1.0,
        );
        let down = (floor_from_camera.rotation * ray).z;
        let depth = -floor_from_camera.translation.vector.z / down;
        let hit = floor_from_camera * Point3::from(ray * depth);
        (down < 0.0).then_some((depth, (1.5..1.8).contains(&hit.x) && hit.y.abs() < 0.2))
    }

    /// The model's depth for every pixel: `on_floor` and `in_patch` scale the true floor depth, 20 m where there is no floor.
    fn predicted(
        map: &UndistortMap,
        floor_from_camera: &Isometry3<f64>,
        depth_of: impl Fn(f64, bool) -> f64,
    ) -> Vec<f32> {
        (0..map.height)
            .flat_map(|row| (0..map.width).map(move |column| (column, row)))
            .map(
                |(column, row)| match floor_hit(map, floor_from_camera, column, row) {
                    Some((depth, in_patch)) => depth_of(depth, in_patch) as f32,
                    None => 20.0,
                },
            )
            .collect()
    }

    fn prior(min_height: f64) -> FloorPrior {
        FloorPrior {
            step: 4,
            tolerance: 0.05,
            min_height,
            rounds: 3,
            max_range: 6.0,
            snap: snap(0.0, 0.0),
        }
    }

    /// HeadFloorPrior's defaults: snap from 15 cm under to 5 cm over the floor, footprint and view fill as given.
    fn snap(footprint_radius: f64, view_fill_reach: f64) -> FloorSnap {
        FloorSnap {
            tolerance: 0.05,
            min_height: -0.15,
            footprint_radius,
            footprint_spacing: 0.05,
            view_fill_reach,
            view_fill_bin_deg: 5.0,
        }
    }

    #[test]
    fn points_near_the_floor_slide_onto_it_and_obstacles_and_drops_do_not() {
        let (map, floor_from_camera) = camera();
        // Floor rays stopped (or carried on) until they met a plane offset from the floor: (offset, slides).
        for (offset, slides) in [(0.03, true), (-0.1, true), (0.1, false), (-0.3, false)] {
            let pin = &map.camera;
            let points: Vec<[f32; 3]> = (0..map.height)
                .step_by(8)
                .flat_map(|row| (0..map.width).step_by(8).map(move |column| (column, row)))
                .filter_map(|(column, row)| {
                    let (depth, _) = floor_hit(&map, &floor_from_camera, column, row)?;
                    let ray = Vector3::new(
                        (column as f64 - pin.cx as f64) / pin.fx as f64,
                        (row as f64 - pin.cy as f64) / pin.fy as f64,
                        1.0,
                    );
                    // Camera 1 m up: reaching the plane `offset` from the floor scales the floor depth by (1 - offset).
                    let point = ray * depth * (1.0 - offset);
                    (depth < 6.0).then_some([point.x as f32, point.y as f32, point.z as f32])
                })
                .collect();
            assert!(points.len() > 50);

            let moved = snap(0.0, 0.0).apply(points, &floor_from_camera);

            for [x, y, z] in moved {
                let height = (floor_from_camera * Point3::new(x as f64, y as f64, z as f64)).z;
                let expected = if slides { 0.0 } else { offset };
                assert!(
                    (height - expected).abs() < 1e-4,
                    "offset {offset}: {height}"
                );
            }
        }
    }

    #[test]
    fn the_footprint_is_a_disc_of_floor() {
        let under = snap(0.325, 0.0).footprint();
        assert!(under.iter().all(|&[x, y]| x.hypot(y) <= 0.325));
        let disc = std::f64::consts::PI * 0.325f64.powi(2) / 0.05f64.powi(2);
        assert!(
            (under.len() as f64 - disc).abs() < 0.15 * disc,
            "{}",
            under.len()
        );
    }

    #[test]
    fn unseen_floor_is_filled_only_where_the_nearest_thing_seen_is_floor() {
        // Floor seen 0.8 m out on bearings 0-30 deg, a box 0.6 m out on 40-50 deg with floor behind it; nothing seen behind.
        let at = |radius: f64, degrees: f64, height: f64| {
            let angle = degrees.to_radians();
            Point3::new(radius * angle.cos(), radius * angle.sin(), height)
        };
        let seen: Vec<Point3<f64>> = (0..=30)
            .map(|degrees| at(0.8, degrees as f64, 0.0))
            .chain((40..=50).map(|degrees| at(0.6, degrees as f64, 0.3)))
            .chain((40..=50).map(|degrees| at(0.8, degrees as f64, 0.0)))
            .collect();

        let filled = snap(0.325, 1.5).view_fill(&seen);

        assert!(!filled.is_empty());
        for &[x, y] in &filled {
            let (radius, bearing) = (x.hypot(y), y.atan2(x).to_degrees());
            assert!(radius > 0.325 && radius < 0.8, "{radius}");
            assert!((-0.01..35.0).contains(&bearing), "{bearing}");
        }
        assert!(filled.iter().any(|&[x, y]| {
            let bearing = y.atan2(x).to_degrees();
            (10.0..15.0).contains(&bearing) && x.hypot(y) > 0.7
        }));
    }

    #[test]
    fn floor_read_too_far_is_anchored_on_the_floor_and_a_box_on_it_is_not() {
        let (map, floor_from_camera) = camera();
        // The model reads the floor 30% too far and the box as a block 20% nearer than the floor behind it.
        let pred = predicted(&map, &floor_from_camera, |depth, on_box| {
            depth * if on_box { 0.8 } else { 1.3 }
        });
        let calibration = Calibration::new(CalibrationConfig::default());

        let floor = prior(-1.0).floor_anchors(&calibration, &pred, &map, &[], &floor_from_camera);

        assert!(floor.len() > 100, "{}", floor.len());
        for anchor in &floor {
            let (depth, on_box) = floor_hit(
                &map,
                &floor_from_camera,
                anchor.u as usize,
                anchor.v as usize,
            )
            .unwrap();
            assert!(!on_box, "anchored the box at {anchor:?}");
            assert!(
                (anchor.z as f64 - depth).abs() < 1e-4,
                "{anchor:?} vs {depth}"
            );
        }
    }

    #[test]
    fn a_drop_deeper_than_min_height_is_not_anchored_on_the_floor() {
        let (map, floor_from_camera) = camera();
        // The floor read right, and a pit 30 cm deep in the patch: its rays run on to z = -0.3.
        let pred = predicted(&map, &floor_from_camera, |depth, in_pit| {
            depth * if in_pit { 1.3 } else { 1.0 }
        });
        let calibration = Calibration::new(CalibrationConfig::default());

        let floor = prior(-0.15).floor_anchors(&calibration, &pred, &map, &[], &floor_from_camera);

        assert!(floor.len() > 100, "{}", floor.len());
        for anchor in &floor {
            let (_, in_pit) = floor_hit(
                &map,
                &floor_from_camera,
                anchor.u as usize,
                anchor.v as usize,
            )
            .unwrap();
            assert!(!in_pit, "anchored the pit at {anchor:?}");
        }
    }
}
