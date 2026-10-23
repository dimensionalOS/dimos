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

//! Generated CDR and TF boundary adapters for the numeric voxel mapper.

use dimos_generated_messages::sensor_msgs::msg::{
    point_cloud2::PointCloud2, point_field::PointField,
};

use dimos_module::pointcloud::{extract_xyz_rows, ExtractError};

pub fn extract_xyz(cloud: &PointCloud2) -> Result<Vec<[f32; 3]>, ExtractError> {
    let offset = |name: &str| {
        cloud
            .fields
            .iter()
            .find(|f| f.name == name && f.datatype == PointField::FLOAT32 && f.count == 1)
            .map(|f| f.offset as usize)
            .ok_or(ExtractError("missing scalar float32 XYZ field"))
    };
    extract_xyz_rows(
        &cloud.data,
        [cloud.width as usize, cloud.height as usize],
        [cloud.point_step as usize, cloud.row_step as usize],
        [offset("x")?, offset("y")?, offset("z")?],
        cloud.is_bigendian,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_messages::builtin_interfaces::msg::time::Time;
    use dimos_generated_messages::std_msgs::msg::header::Header;

    #[test]
    fn organized_cloud_reader_keeps_endian_and_row_padding_and_filters_missing_points() {
        let mut data = vec![0; 80];
        for (offset, point) in [
            (0, [1_f32, 2., 3.]),
            (16, [4., 5., 6.]),
            (40, [7., 8., 9.]),
            (56, [f32::NAN, 0., 0.]),
        ] {
            for (i, value) in point.iter().enumerate() {
                data[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&value.to_be_bytes());
            }
        }
        let mut cloud = PointCloud2 {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "lidar".into(),
            },
            height: 2,
            width: 2,
            fields: ["x", "y", "z"]
                .into_iter()
                .enumerate()
                .map(|(i, n)| PointField {
                    name: n.into(),
                    offset: (i * 4) as u32,
                    datatype: PointField::FLOAT32,
                    count: 1,
                })
                .collect(),
            is_bigendian: true,
            point_step: 16,
            row_step: 40,
            data,
            is_dense: false,
        };
        assert_eq!(
            extract_xyz(&cloud).unwrap(),
            vec![[1., 2., 3.], [4., 5., 6.], [7., 8., 9.]]
        );
        cloud.row_step = 12;
        assert!(extract_xyz(&cloud).is_err());
    }
}
