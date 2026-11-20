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
//! The M20's body IMU (a Yesense on the AOS board's UART) read straight off the serial port as `sensor_msgs/Imu`,
//! every frame, with base_link -> imu frame published alongside.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_module::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use dimos_module::{native_config, Module, Output, Tf, Transform};
use lcm_msgs::sensor_msgs::Imu;
use tokio::runtime::Handle;
use tracing::{info, warn};
use yesense_imu::{DecoderConfig, Imu as YesenseImu};

use crate::mount::{imu_message, mount_rotation};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Serial device and rate the Yesense streams on.
    device: String,
    #[validate(range(min = 1, max = 4_294_967_295_i64))]
    baud: i64,
    /// The frame the samples are published in, and its parent.
    frame_id: String,
    parent_frame_id: String,
    /// The sensor's position in base_link.
    imu_xyz: [f64; 3],
    /// The sensor's roll, pitch, yaw in base_link.
    mount_rpy_rad: [f64; 3],
    /// Rotate samples into base_link's axes (as the vendor driver does), so the published frame is base_link-aligned.
    body_axes: bool,
    /// Bytes asked for per read.
    #[validate(range(min = 1))]
    read_chunk_bytes: i64,
    /// Bytes the decoder holds waiting for a frame before dropping the oldest.
    #[validate(range(min = 7))]
    max_buffered_bytes: i64,
    /// Seconds between attempts to (re)open the device.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
    /// Seconds between publishes of the static base_link -> imu transform.
    #[validate(range(min = 0.01, max = 60.0))]
    tf_period_s: f64,
}

#[derive(Module)]
#[module(name = "m20_imu", setup = start)]
pub struct M20Imu {
    #[output(encode = Imu::encode)]
    imu: Output<Imu>,
    #[tf]
    tf: Tf,

    #[config]
    config: Config,
}

impl M20Imu {
    async fn start(&mut self) {
        let config = self.config.clone();
        let output = self.imu.clone();
        let handle = Handle::current();
        let reader_config = config.clone();
        std::thread::spawn(move || read_forever(reader_config, output, handle));
        tokio::spawn(publish_mount(config, self.tf.clone()));
    }
}

fn wall_s(time: SystemTime) -> f64 {
    time.duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

/// Open the device, publish every frame stamped with its host receive time, reopen after an error.
fn read_forever(config: Config, output: Output<Imu>, handle: Handle) {
    let mount = mount_rotation(config.mount_rpy_rad);
    // the validator bounds these; a value that still does not fit falls back to the smallest legal one
    let serial = yesense_imu::Config {
        path: config.device.clone(),
        baud: u32::try_from(config.baud).unwrap_or(1),
        read_chunk_bytes: usize::try_from(config.read_chunk_bytes).unwrap_or(1),
        decoder: DecoderConfig {
            max_buffered_bytes: usize::try_from(config.max_buffered_bytes).unwrap_or(0),
        },
    };
    loop {
        match YesenseImu::open(&serial) {
            Ok(mut imu) => {
                info!(device = %config.device, baud = config.baud, "reading the IMU");
                let error = loop {
                    match imu.next_sample() {
                        Ok(sample) => {
                            let msg = imu_message(
                                &sample.frame,
                                &mount,
                                config.body_axes,
                                &config.frame_id,
                                wall_s(sample.host_time),
                            );
                            if let Err(error) = handle.block_on(output.publish(&msg)) {
                                warn!(%error, "cannot publish the IMU sample");
                            }
                        }
                        Err(error) => break error,
                    }
                };
                warn!(%error, stats = ?imu.stats(), "IMU read failed, reopening");
            }
            Err(error) => warn!(%error, device = %config.device, "cannot open the IMU"),
        }
        // retry backoff, not load-bearing: the UART node is fixed, so there is no appearance to watch for
        std::thread::sleep(Duration::from_secs_f64(config.retry_s));
    }
}

/// parent -> imu frame: the sensor's position, and its mount rotation unless samples are already in body axes.
async fn publish_mount(config: Config, tf: Tf) {
    let [x, y, z] = config.imu_xyz;
    let rotation = if config.body_axes {
        UnitQuaternion::identity()
    } else {
        mount_rotation(config.mount_rpy_rad)
    };
    let pose = Isometry3::from_parts(Translation3::new(x, y, z), rotation);
    let mut ticks = tokio::time::interval(Duration::from_secs_f64(config.tf_period_s));
    loop {
        ticks.tick().await;
        let transform = Transform::new(
            config.parent_frame_id.clone(),
            config.frame_id.clone(),
            wall_s(SystemTime::now()),
            pose,
        );
        if let Err(error) = tf.publish(&[transform]).await {
            warn!(%error, "cannot publish the IMU transform");
        }
    }
}
