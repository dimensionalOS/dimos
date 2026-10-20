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

//! Generated CDR and TF boundary adapters for the numeric MLS planner.

use dimos_generated_messages::builtin_interfaces::msg::time::Time;
use dimos_generated_messages::std_msgs::msg::header::Header;
use dimos_module::cdr;

pub fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
    cdr::encode(value).expect("serialize a valid generated map value")
}

fn nanoseconds(time: &Time) -> i64 {
    time.sec as i64 * 1_000_000_000 + time.nanosec as i64
}

pub fn decode_tf(bytes: &[u8]) -> std::io::Result<Vec<dimos_module::tf::Transform>> {
    use dimos_generated_messages::tf2_msgs::msg::tf_message::TFMessage;
    use dimos_module::nalgebra::{Quaternion, Translation3, UnitQuaternion};
    let message: TFMessage = cdr::decode(bytes)?;
    message
        .transforms
        .into_iter()
        .map(|edge| {
            let p = edge.transform.translation;
            let q = edge.transform.rotation;
            if ![p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "nonfinite TF components",
                ));
            }
            let rotation = UnitQuaternion::try_new(Quaternion::new(q.w, q.x, q.y, q.z), 1e-9)
                .ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidData, "zero TF quaternion")
                })?;
            if edge.header.stamp.nanosec >= 1_000_000_000 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "unnormalized TF stamp",
                ));
            }
            Ok(dimos_module::tf::Transform::from_nanoseconds(
                edge.header.frame_id,
                edge.child_frame_id,
                nanoseconds(&edge.header.stamp),
                dimos_module::nalgebra::Isometry3::from_parts(
                    Translation3::new(p.x, p.y, p.z),
                    rotation,
                ),
            ))
        })
        .collect()
}

pub fn encode_tf(transforms: &[dimos_module::tf::Transform]) -> std::io::Result<Vec<u8>> {
    use dimos_generated_messages::{
        geometry_msgs::msg::{
            quaternion::Quaternion, transform::Transform, transform_stamped::TransformStamped,
            vector3::Vector3,
        },
        tf2_msgs::msg::tf_message::TFMessage,
    };
    let transforms = transforms
        .iter()
        .map(|value| {
            let ns = value.stamp_nanoseconds().ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "TF timestamp is outside range",
                )
            })?;
            let sec = i32::try_from(ns.div_euclid(1_000_000_000)).map_err(|_| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "TF seconds exceed ROS int32",
                )
            })?;
            let p = value.translation();
            let q = value.rotation();
            Ok(TransformStamped {
                header: Header {
                    stamp: Time {
                        sec,
                        nanosec: ns.rem_euclid(1_000_000_000) as u32,
                    },
                    frame_id: value.parent.clone(),
                },
                child_frame_id: value.child.clone(),
                transform: Transform {
                    translation: Vector3 {
                        x: p.x,
                        y: p.y,
                        z: p.z,
                    },
                    rotation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
            })
        })
        .collect::<std::io::Result<Vec<_>>>()?;
    cdr::encode(&TFMessage { transforms })
}
