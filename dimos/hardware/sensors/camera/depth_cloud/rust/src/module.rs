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

use std::time::Duration;

use dimos_module::{native_config, warn_throttled, Input, Module, Output};
use lcm_msgs::sensor_msgs::{CameraInfo, Image, PointCloud2, PointField};
use lcm_msgs::std_msgs::Header;

use crate::unproject::{unproject, Params, POINT_STEP};

#[native_config]
pub struct Config {
    /// Keep every Nth pixel on each axis. A 1280x720 depth frame is 920k
    /// points; at 4 that is 57k, the range the voxel map ray-caster is sized for.
    #[validate(range(min = 1, max = 64))]
    decimation: i64,
    /// Depth below this is the sensor's blind zone; above it, stereo range error
    /// grows past the voxel size and smears obstacles.
    #[validate(range(min = 0.0, max = 1000.0))]
    min_range_m: f64,
    #[validate(range(min = 0.0, max = 1000.0))]
    max_range_m: f64,
    /// Multiplier onto metres. uint16 depth is millimetres, so 0.001; float32
    /// depth is already metres and ignores this.
    #[validate(range(min = 0.0, max = 1000.0))]
    depth_scale: f64,
    /// Overrides the frame the cloud is published in. Empty defers to the
    /// CameraInfo, then to the depth image.
    frame_id: String,
}

#[derive(Module)]
#[module(name = "depth_cloud")]
pub struct DepthCloud {
    #[input(decode = Image::decode, handler = on_depth)]
    depth: Input<Image>,

    #[input(decode = CameraInfo::decode, handler = on_camera_info)]
    camera_info: Input<CameraInfo>,

    #[output(encode = PointCloud2::encode)]
    cloud: Output<PointCloud2>,

    #[config]
    config: Config,

    /// Latest intrinsics. Only a depth frame drives a cloud: intrinsics are
    /// effectively static and are republished purely so a late consumer sees
    /// them, so pairing the two streams would emit a duplicate per republish.
    info: Option<CameraInfo>,
}

impl DepthCloud {
    async fn on_camera_info(&mut self, msg: CameraInfo) {
        self.info = Some(msg);
    }

    async fn on_depth(&mut self, msg: Image) {
        // Depth arriving before the intrinsics has no valid unprojection, the
        // same way the first frames off a real camera do not.
        let Some(info) = self.info.as_ref() else {
            return;
        };

        let params = Params {
            decimation: self.config.decimation as usize,
            min_range_m: self.config.min_range_m as f32,
            max_range_m: self.config.max_range_m as f32,
            depth_scale: self.config.depth_scale as f32,
        };
        let (data, count) = match unproject(&msg, info, &params) {
            Ok(points) => points,
            Err(error) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %error,
                    "Could not unproject a depth frame, dropped it.",
                );
                return;
            }
        };

        let frame_id = resolve_frame_id(
            &self.config.frame_id,
            &info.header.frame_id,
            &msg.header.frame_id,
        )
        .to_owned();

        let cloud = make_cloud(data, count, frame_id, msg.header);
        self.cloud.publish(&cloud).await.ok();
    }
}

/// A vendor driver often stamps depth with an optical frame nobody publishes a
/// transform for, while the intrinsics can carry the link the robot actually
/// puts on tf. So the calibration frame outranks the image's own.
fn resolve_frame_id<'a>(config: &'a str, info: &'a str, depth: &'a str) -> &'a str {
    [config, info, depth]
        .into_iter()
        .find(|candidate| !candidate.is_empty())
        .unwrap_or_default()
}

fn make_cloud(data: Vec<u8>, count: i32, frame_id: String, header: Header) -> PointCloud2 {
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: Header {
            seq: header.seq,
            stamp: header.stamp,
            frame_id,
        },
        height: 1,
        width: count,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: POINT_STEP as i32,
        row_step: POINT_STEP as i32 * count,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::resolve_frame_id;

    #[test]
    fn the_calibration_frame_outranks_the_vendors_optical_one() {
        assert_eq!(
            resolve_frame_id("", "camera_head_left_link", "vendor_optical"),
            "camera_head_left_link"
        );
    }

    #[test]
    fn the_config_overrides_both() {
        assert_eq!(
            resolve_frame_id("override", "camera_head_left_link", "vendor_optical"),
            "override"
        );
    }

    #[test]
    fn unlabelled_intrinsics_fall_back_to_the_depth_frame() {
        assert_eq!(resolve_frame_id("", "", "vendor_optical"), "vendor_optical");
    }
}
