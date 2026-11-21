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

//! The dock detector as a dimos module (reflective_column_node's replacement): one lidar's sweeps in, the dock's
//! pose in base_link out on `dock_pose` for every sweep that shows both posts (none when it does not: the controller's
//! docking counts how long it has gone without one). It runs next to the lidars so full sweeps never cross to the
//! controller's board; the controller gets ~10 small poses a second.

use std::time::Duration;

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::Header;
use tracing::{info, warn};

use super::detector::{detect, Detection, Mount};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// only sweeps in this frame are searched (the M20's lidar stream carries both Airys)
    lidar_frame_id: String,
    /// that lidar's mount in base_link (m; roll, pitch, yaw rad)
    mount_xyz: [f64; 3],
    mount_rpy: [f64; 3],
    /// an m20_controller.toml path for [dock.detector]; empty uses the one built in
    controller_config: String,
    /// dock_pose's frame
    frame_id: String,
}

#[derive(Module)]
#[module(name = "m20_dock_detector", setup = start)]
pub struct M20DockDetector {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,
    #[output(encode = PoseStamped::encode)]
    dock_pose: Output<PoseStamped>,
    #[config]
    config: Config,
    detector: Option<(super::config::DetectorConfig, Mount)>,
    /// the dock in sight on the last sweep (logged on a change)
    seen: Option<bool>,
}

/// (x, y, z, intensity) of every point, or why the cloud cannot be read.
pub fn cloud_points(cloud: &PointCloud2) -> Result<Vec<[f32; 4]>, String> {
    if cloud.is_bigendian {
        return Err("big-endian clouds are not supported".into());
    }
    let step = cloud.point_step as usize;
    let offset = |name: &str| -> Result<usize, String> {
        let f = cloud
            .fields
            .iter()
            .find(|f| f.name == name)
            .ok_or_else(|| format!("cloud has no {name} field"))?;
        if f.datatype != PointField::FLOAT32 as u8 {
            return Err(format!(
                "field {name} is datatype {}, want FLOAT32",
                f.datatype
            ));
        }
        let o = f.offset as usize;
        if o + 4 > step {
            return Err(format!("field {name} at {o} overruns point_step {step}"));
        }
        Ok(o)
    };
    let fields = [
        offset("x")?,
        offset("y")?,
        offset("z")?,
        offset("intensity")?,
    ];
    let n = cloud.width as usize * cloud.height as usize;
    if step == 0 || cloud.data.len() < n * step {
        return Err(format!(
            "{} bytes for {n} points of {step}",
            cloud.data.len()
        ));
    }
    let f32_at = |p: &[u8], o: usize| {
        p.get(o..o + 4)
            .and_then(|b| b.try_into().ok())
            .map_or(f32::NAN, f32::from_le_bytes)
    };
    Ok(cloud
        .data
        .chunks_exact(step)
        .take(n)
        .map(|p| fields.map(|o| f32_at(p, o)))
        .collect())
}

/// The dock's pose as a PoseStamped: position (x, y, 0), yaw about z.
pub fn pose_message(header: &Header, frame_id: &str, x: f64, y: f64, yaw: f64) -> PoseStamped {
    PoseStamped {
        header: Header {
            seq: header.seq,
            stamp: header.stamp.clone(),
            frame_id: frame_id.to_string(),
        },
        pose: Pose {
            position: Point { x, y, z: 0.0 },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: (yaw / 2.0).sin(),
                w: (yaw / 2.0).cos(),
            },
        },
    }
}

impl M20DockDetector {
    async fn start(&mut self) {
        let cfg = match crate::module::load_controller_config(&self.config.controller_config) {
            Ok(cfg) => cfg,
            Err(error) => {
                warn!(
                    error = format!("{error:#}"),
                    "bad controller config; detecting nothing"
                );
                return;
            }
        };
        let mount = Mount::from_xyz_rpy(self.config.mount_xyz, self.config.mount_rpy);
        info!(frame = %self.config.lidar_frame_id, ?mount, "looking for the dock's reflector posts");
        self.detector = Some((cfg.dock.detector, mount));
    }

    async fn on_lidar(&mut self, cloud: PointCloud2) {
        if cloud.header.frame_id != self.config.lidar_frame_id {
            return;
        }
        let Some((cfg, mount)) = &self.detector else {
            return;
        };
        let points = match cloud_points(&cloud) {
            Ok(points) => points,
            Err(error) => {
                dimos_module::warn_throttled!(Duration::from_secs(5), %error, "dropping cloud");
                return;
            }
        };
        let detection = detect(points, mount, cfg);
        let seen = matches!(detection, Detection::Found { .. });
        if self.seen != Some(seen) {
            match &detection {
                Detection::Found {
                    pose,
                    posts,
                    points,
                } => {
                    info!(?pose, ?posts, ?points, "dock in sight")
                }
                Detection::Missing(why) => info!(%why, "dock not in sight"),
            }
            self.seen = Some(seen);
        }
        if let Detection::Found { pose, .. } = detection {
            let msg = pose_message(
                &cloud.header,
                &self.config.frame_id,
                pose.x,
                pose.y,
                pose.yaw,
            );
            if let Err(error) = self.dock_pose.publish(&msg).await {
                warn!(%error, "cannot publish the dock pose");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_the_airy_full_layout_and_refuses_bad_ones() {
        let field = |name: &str, offset: i32, datatype: i8| PointField {
            name: name.into(),
            offset,
            datatype: datatype as u8,
            count: 1,
        };
        let mut data = Vec::new();
        for v in [1.0f32, 2.0, 3.0, 250.0] {
            data.extend_from_slice(&v.to_le_bytes());
        }
        data.extend_from_slice(&[0u8; 6]);
        let mut cloud = PointCloud2 {
            header: Header::default(),
            height: 1,
            width: 1,
            fields: vec![
                field("x", 0, PointField::FLOAT32),
                field("y", 4, PointField::FLOAT32),
                field("z", 8, PointField::FLOAT32),
                field("intensity", 12, PointField::FLOAT32),
                field("offset_time", 16, PointField::UINT32),
                field("ring", 20, PointField::UINT16),
            ],
            is_bigendian: false,
            point_step: 22,
            row_step: 22,
            data,
            is_dense: true,
        };
        assert_eq!(cloud_points(&cloud).unwrap(), vec![[1.0, 2.0, 3.0, 250.0]]);
        cloud.width = 2;
        assert!(cloud_points(&cloud).is_err(), "more points than data");
        cloud.width = 1;
        cloud.fields.retain(|f| f.name != "intensity");
        assert!(cloud_points(&cloud).is_err(), "no intensity");
    }

    #[test]
    fn pose_message_round_trips_the_yaw() {
        let m = pose_message(&Header::default(), "base_link", 1.0, -0.2, 3.0);
        let q = &m.pose.orientation;
        let yaw = 2.0 * q.z.atan2(q.w);
        assert!((yaw - 3.0).abs() < 1e-12);
        assert_eq!(m.header.frame_id, "base_link");
    }
}
