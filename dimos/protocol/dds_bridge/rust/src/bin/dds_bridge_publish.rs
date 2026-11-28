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

//! Publish synthetic ROS 2 samples on a DDS topic, sample `i` carrying `i` in a known field.
//!
//! `dds_bridge_publish <domain_id> <dds_topic> <ros_type> <count>` waits for a reader to
//! match, writes `count` reliable samples, and waits for them to be acknowledged.

use std::process::exit;
use std::time::Duration;

use dimos_dds_bridge::ros::*;
use dimos_dds_bridge::translate::ros2_dds_type_name;
use rustdds::no_key::DataWriter;
use rustdds::policy::{History, Reliability};
use rustdds::{
    CDRSerializerAdapter, DataWriterStatus, DomainParticipant, QosPolicyBuilder, StatusEvented,
    TopicKind,
};
use serde::Serialize;

/// How often to look for a matched reader.
const MATCH_POLL: Duration = Duration::from_millis(50);
/// How long to wait for the reader to acknowledge every sample.
const ACK_WAIT: Duration = Duration::from_secs(10);

fn header(i: i32, frame_id: &str) -> Header {
    Header {
        stamp: Time {
            sec: i,
            nanosec: 500_000_000,
        },
        frame_id: frame_id.to_string(),
    }
}

fn identity() -> Quaternion {
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }
}

fn point_cloud(i: i32) -> PointCloud2 {
    let points = (i + 1) as u32;
    let fields = ["x", "y", "z"]
        .iter()
        .enumerate()
        .map(|(k, name)| PointField {
            name: name.to_string(),
            offset: 4 * k as u32,
            datatype: 7,
            count: 1,
        })
        .collect();
    let mut data = Vec::new();
    for p in 0..points {
        for axis in 1..=3 {
            data.extend_from_slice(&((p * axis) as f32).to_le_bytes());
        }
    }
    PointCloud2 {
        header: header(i, "lidar"),
        height: 1,
        width: points,
        fields,
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points,
        data: serde_bytes::ByteBuf::from(data),
        is_dense: true,
    }
}

fn odometry(i: i32) -> Odometry {
    let mut pose_covariance = [0.0; 36];
    pose_covariance[0] = 0.1;
    Odometry {
        header: header(i, "odom"),
        child_frame_id: "base_link".to_string(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: i as f64,
                    y: 2.0,
                    z: 0.0,
                },
                orientation: identity(),
            },
            covariance: pose_covariance,
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: 1.5,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3::default(),
            },
            covariance: [0.0; 36],
        },
    }
}

fn transform(i: i32, parent: &str, child: &str, x: f64) -> TransformStamped {
    TransformStamped {
        header: header(i, parent),
        child_frame_id: child.to_string(),
        transform: Transform {
            translation: Vector3 { x, y: 0.0, z: 0.0 },
            rotation: identity(),
        },
    }
}

fn publish<T: Serialize + 'static>(
    participant: &DomainParticipant,
    dds_topic: &str,
    ros_type: &str,
    count: i32,
    sample: fn(i32) -> T,
) {
    let qos = QosPolicyBuilder::new()
        .reliability(Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_secs(1),
        })
        .history(History::KeepAll)
        .build();
    let type_name = ros2_dds_type_name(ros_type).expect("a pkg/msg/Name type");
    let topic = participant
        .create_topic(dds_topic.to_string(), type_name, &qos, TopicKind::NoKey)
        .expect("DDS topic");
    let publisher = participant.create_publisher(&qos).expect("DDS publisher");
    let writer: DataWriter<T, CDRSerializerAdapter<T>> = publisher
        .create_datawriter_no_key(&topic, None)
        .expect("DDS writer");
    eprintln!("[publish] waiting for a reader on {dds_topic}");
    loop {
        match writer.try_recv_status() {
            Some(DataWriterStatus::PublicationMatched { .. }) => break,
            Some(_) => {}
            None => std::thread::sleep(MATCH_POLL),
        }
    }
    for i in 0..count {
        if writer.write(sample(i), None).is_err() {
            eprintln!("[publish] write {i} failed");
            exit(1);
        }
    }
    let acknowledged = writer.wait_for_acknowledgments(ACK_WAIT);
    eprintln!("[publish] wrote {count} samples on {dds_topic}, acknowledged: {acknowledged:?}");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 5 {
        eprintln!(
            "usage: {} <domain_id> <dds_topic> <ros_type> <count>",
            args[0]
        );
        exit(2);
    }
    let domain_id: u16 = args[1].parse().expect("domain_id is a u16");
    let (dds_topic, ros_type) = (args[2].as_str(), args[3].as_str());
    let count: i32 = args[4].parse().expect("count is an integer");
    let participant = DomainParticipant::new(domain_id).expect("join the DDS domain");
    let p = &participant;
    match ros_type {
        "std_msgs/msg/Int32" => publish(p, dds_topic, ros_type, count, |i| Int32 { data: i }),
        "std_msgs/msg/UInt32" => {
            publish(p, dds_topic, ros_type, count, |i| UInt32 { data: i as u32 })
        }
        "std_msgs/msg/Bool" => {
            publish(p, dds_topic, ros_type, count, |i| Bool { data: i % 2 == 1 })
        }
        "geometry_msgs/msg/Twist" => publish(p, dds_topic, ros_type, count, |i| Twist {
            linear: Vector3 {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        }),
        "geometry_msgs/msg/PoseStamped" => {
            publish(p, dds_topic, ros_type, count, |i| PoseStamped {
                header: header(i, "map"),
                pose: Pose {
                    position: Point {
                        x: i as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: identity(),
                },
            })
        }
        "nav_msgs/msg/Odometry" => publish(p, dds_topic, ros_type, count, odometry),
        "sensor_msgs/msg/PointCloud2" => publish(p, dds_topic, ros_type, count, point_cloud),
        "sensor_msgs/msg/Imu" => publish(p, dds_topic, ros_type, count, |i| Imu {
            header: header(i, "imu"),
            orientation: identity(),
            angular_velocity: Vector3 {
                x: 0.0,
                y: 0.0,
                z: i as f64,
            },
            linear_acceleration: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 9.81,
            },
            ..Imu::default()
        }),
        "tf2_msgs/msg/TFMessage" => publish(p, dds_topic, ros_type, count, |i| TfMessage {
            transforms: vec![
                transform(i, "odom", "base_link", i as f64),
                transform(i, "base_link", "lidar", 0.2),
            ],
        }),
        other => {
            eprintln!("no synthetic sample for {other:?}");
            exit(2);
        }
    }
}
