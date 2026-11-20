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
//! The M20's body IMU as `sensor_msgs/Imu`, every sample the vendor yesense_node publishes (already in base_link's
//! axes, stamped by the vendor), copied passively off its DDS traffic, plus the base_link -> imu frame transform.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_module::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use dimos_module::{native_config, Module, Output, Tf, Transform};
use lcm_msgs::geometry_msgs;
use lcm_msgs::sensor_msgs::Imu;
use lcm_msgs::std_msgs::{Header, Time};
use tokio::runtime::Handle;
use tracing::warn;

use crate::rtps::ImuSample;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// The interface the vendor samples cross (yesense_node's participant is loopback-only).
    interface: String,
    /// The vendor participant's name in its SPDP announcements.
    participant: String,
    /// The domain's SPDP multicast port, where every participant announces itself.
    #[validate(range(min = 1, max = 65535))]
    discovery_port: i64,
    /// The DDS user-traffic port range to capture.
    #[validate(range(min = 1, max = 65535))]
    port_min: i64,
    #[validate(range(min = 1, max = 65535))]
    port_max: i64,
    /// Datagrams shorter than this (UDP header included) are dropped in the kernel: RTPS heartbeats and acks
    /// (132 bytes) and yesense_node's other 228-byte messages are smaller than an Imu sample (472).
    #[validate(range(min = 0, max = 65535))]
    min_udp_bytes: i64,
    /// The frame the samples are published in, and its parent.
    frame_id: String,
    parent_frame_id: String,
    /// The sensor's position in base_link (the samples are already base_link-aligned).
    imu_xyz: [f64; 3],
    /// The capture socket's receive buffer.
    #[validate(range(min = 65536, max = 2147483647))]
    recv_buffer_bytes: i64,
    /// Seconds between attempts to (re)open the capture.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
    /// Seconds between publishes of the static base_link -> imu transform.
    #[validate(range(min = 0.01, max = 60.0))]
    tf_period_s: f64,
}

#[derive(Module)]
#[module(name = "m20_body_imu_sniff", setup = start)]
pub struct M20BodyImuSniff {
    #[output(encode = Imu::encode)]
    imu: Output<Imu>,
    #[tf]
    tf: Tf,

    #[config]
    config: Config,
}

impl M20BodyImuSniff {
    async fn start(&mut self) {
        let config = self.config.clone();
        let output = self.imu.clone();
        let handle = Handle::current();
        let reader_config = config.clone();
        std::thread::spawn(move || sniff_forever(reader_config, output, handle));
        tokio::spawn(publish_mount(config, self.tf.clone()));
    }
}

/// The vendor sample as an Imu in `frame_id`, keeping its stamp and covariances.
pub fn imu_message(sample: &ImuSample, frame_id: &str) -> Imu {
    let vector = |v: [f64; 3]| geometry_msgs::Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    };
    let [x, y, z, w] = sample.orientation_xyzw;
    Imu {
        header: Header {
            seq: 0,
            stamp: Time {
                sec: sample.stamp_sec,
                nsec: i32::try_from(sample.stamp_nanosec).unwrap_or(i32::MAX),
            },
            frame_id: frame_id.into(),
        },
        orientation: geometry_msgs::Quaternion { x, y, z, w },
        orientation_covariance: sample.orientation_covariance,
        angular_velocity: vector(sample.angular_velocity),
        angular_velocity_covariance: sample.angular_velocity_covariance,
        linear_acceleration: vector(sample.linear_acceleration),
        linear_acceleration_covariance: sample.linear_acceleration_covariance,
    }
}

#[cfg(target_os = "linux")]
fn sniff_forever(config: Config, output: Output<Imu>, handle: Handle) {
    use crate::capture::{wait_readable, SniffCaptures};
    use crate::rtps::Sniffer;
    use tracing::info;

    loop {
        let mut sniffer = Sniffer::new(&config.participant);
        // the validator bounds these to their ranges, so the fallbacks never apply
        let port = |value: i64| u16::try_from(value).unwrap_or(u16::MAX);
        let opened = SniffCaptures::open(
            &config.interface,
            port(config.discovery_port),
            port(config.port_min),
            port(config.port_max),
            port(config.min_udp_bytes),
            usize::try_from(config.recv_buffer_bytes).unwrap_or(0),
        );
        let error = match opened {
            Ok(mut captures) => {
                info!(interface = %config.interface, participant = %config.participant, "capturing the vendor IMU samples");
                loop {
                    let [discovery_ready, data_ready] =
                        match wait_readable([&captures.discovery, &captures.data], None) {
                            Ok(ready) => ready,
                            Err(error) => break error,
                        };
                    if discovery_ready {
                        let known = sniffer.prefix();
                        match captures.discovery.next_udp_payload() {
                            Ok(payload) => drop(sniffer.feed(payload)),
                            Err(error) => break error,
                        }
                        if let (Some(prefix), true) = (sniffer.prefix(), known != sniffer.prefix())
                        {
                            info!(?prefix, "found the vendor IMU participant");
                            if let Err(error) = captures.follow(prefix) {
                                break error;
                            }
                        }
                    }
                    if data_ready {
                        let payload = match captures.data.next_udp_payload() {
                            Ok(payload) => payload,
                            Err(error) => break error,
                        };
                        for sample in sniffer.feed(payload) {
                            if let Err(error) = handle
                                .block_on(output.publish(&imu_message(&sample, &config.frame_id)))
                            {
                                warn!(%error, "cannot publish the IMU sample");
                            }
                        }
                    }
                }
            }
            Err(error) => error,
        };
        warn!(%error, "capture failed (it needs CAP_NET_RAW), reopening");
        // retry backoff, not load-bearing: a missing capability or interface has no event to watch for
        std::thread::sleep(Duration::from_secs_f64(config.retry_s));
    }
}

#[cfg(not(target_os = "linux"))]
fn sniff_forever(_config: Config, _output: Output<Imu>, _handle: Handle) {
    warn!("the body IMU sniffer captures with AF_PACKET, which only Linux has");
}

/// parent -> imu frame: the sensor's position, no rotation (the vendor publishes in base_link's axes).
async fn publish_mount(config: Config, tf: Tf) {
    let [x, y, z] = config.imu_xyz;
    let pose = Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity());
    let mut ticks = tokio::time::interval(Duration::from_secs_f64(config.tf_period_s));
    loop {
        ticks.tick().await;
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        let transform = Transform::new(
            config.parent_frame_id.clone(),
            config.frame_id.clone(),
            now,
            pose,
        );
        if let Err(error) = tf.publish(&[transform]).await {
            warn!(%error, "cannot publish the IMU transform");
        }
    }
}
