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

use std::collections::HashMap;
use std::io;
use std::net::Ipv4Addr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use dimos_lcm::{Lcm, LcmOptions};
use lcm_msgs::sensor_msgs::{Image as LcmImage, PointCloud2, PointField};
use regex::Regex;
use rerun::{ChannelDatatype, ColorModel, RecordingStream};
use serde::Deserialize;
use tokio::sync::Notify;

const IMAGE_LCM_SUFFIX: &str = "#sensor_msgs.Image";
const IMAGE_ZENOH_SUFFIX: &str = "/sensor_msgs.Image";
const POINTCLOUD_LCM_SUFFIX: &str = "#sensor_msgs.PointCloud2";
const POINTCLOUD_ZENOH_SUFFIX: &str = "/sensor_msgs.PointCloud2";

#[derive(Deserialize)]
struct Envelope {
    config: Config,
}

#[derive(Deserialize)]
struct Config {
    backend: String,
    connect_url: String,
    entity_prefix: String,
    lcm_url: String,
    max_hz: HashMap<String, f64>,
    python_patterns: Vec<String>,
    recording_id: String,
    zenoh_connect: Vec<String>,
    zenoh_listen: Vec<String>,
    zenoh_mode: String,
}

#[derive(Clone, Copy)]
enum Kind {
    Image,
    PointCloud,
}

struct Latest {
    packets: Mutex<HashMap<String, Vec<u8>>>,
    notify: Notify,
}

impl Latest {
    fn push(&self, channel: &str, payload: Vec<u8>) {
        let mut packets = self.packets.lock().unwrap();
        if let Some(slot) = packets.get_mut(channel) {
            *slot = payload;
        } else {
            packets.insert(channel.to_owned(), payload);
        }
        drop(packets);
        self.notify.notify_one();
    }

    async fn take(&self) -> HashMap<String, Vec<u8>> {
        loop {
            self.notify.notified().await;
            let mut packets = self.packets.lock().unwrap();
            if !packets.is_empty() {
                return std::mem::take(&mut *packets);
            }
        }
    }
}

struct Sink {
    attached: HashMap<String, String>,
    entity_prefix: String,
    last_log: HashMap<String, Instant>,
    min_interval: HashMap<String, Duration>,
    python_patterns: Vec<Regex>,
    recording: RecordingStream,
}

impl Sink {
    fn new(config: &Config) -> Result<Self> {
        let recording = rerun::RecordingStreamBuilder::new("dimos")
            .recording_id(config.recording_id.clone())
            .connect_grpc_opts(config.connect_url.clone())?;
        let python_patterns = config
            .python_patterns
            .iter()
            .map(|pattern| Regex::new(pattern))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let min_interval = config
            .max_hz
            .iter()
            .filter(|(_, hz)| **hz > 0.0)
            .map(|(entity, hz)| (entity.clone(), Duration::from_secs_f64(1.0 / hz)))
            .collect();
        Ok(Self {
            attached: HashMap::new(),
            entity_prefix: config.entity_prefix.clone(),
            last_log: HashMap::new(),
            min_interval,
            python_patterns,
            recording,
        })
    }

    fn log(&mut self, channel: &str, payload: Vec<u8>) -> Result<()> {
        let (kind, topic) = classify(channel).unwrap();
        let entity = format!("{}{}", self.entity_prefix, topic);
        if self
            .python_patterns
            .iter()
            .any(|pattern| pattern.is_match(&entity))
        {
            return Ok(());
        }
        if let Some(interval) = self.min_interval.get(&entity) {
            let now = Instant::now();
            if self
                .last_log
                .get(&entity)
                .is_some_and(|last| now.duration_since(*last) < *interval)
            {
                return Ok(());
            }
            self.last_log.insert(entity.clone(), now);
        }
        let frame_id = match kind {
            Kind::Image => self.log_image(&entity, payload)?,
            Kind::PointCloud => self.log_pointcloud(&entity, payload)?,
        };
        if self.attached.get(&entity) != Some(&frame_id) && !frame_id.is_empty() {
            self.recording.log(
                entity.as_str(),
                &rerun::Transform3D::new().with_parent_frame(format!("tf#/{frame_id}")),
            )?;
            self.attached.insert(entity, frame_id);
        }
        Ok(())
    }

    fn log_image(&self, entity: &str, payload: Vec<u8>) -> Result<String> {
        let image = LcmImage::decode(&payload)?;
        if image.width < 0 || image.height < 0 {
            bail!("Image dimensions must be nonnegative");
        }
        let resolution = [image.width as u32, image.height as u32];
        match image.encoding.as_str() {
            "jpeg" => self
                .recording
                .log(entity, &rerun::EncodedImage::from_file_contents(image.data))?,
            "mono8" => self.recording.log(
                entity,
                &rerun::Image::from_color_model_and_bytes(
                    image.data,
                    resolution,
                    ColorModel::L,
                    ChannelDatatype::U8,
                ),
            )?,
            "mono16" => self.recording.log(
                entity,
                &rerun::Image::from_color_model_and_bytes(
                    image.data,
                    resolution,
                    ColorModel::L,
                    ChannelDatatype::U16,
                ),
            )?,
            "rgb8" => self.log_color(
                entity,
                image.data,
                resolution,
                ColorModel::RGB,
                ChannelDatatype::U8,
            )?,
            "rgba8" => self.log_color(
                entity,
                image.data,
                resolution,
                ColorModel::RGBA,
                ChannelDatatype::U8,
            )?,
            "bgr8" => self.log_color(
                entity,
                image.data,
                resolution,
                ColorModel::BGR,
                ChannelDatatype::U8,
            )?,
            "bgra8" => self.log_color(
                entity,
                image.data,
                resolution,
                ColorModel::BGRA,
                ChannelDatatype::U8,
            )?,
            "32FC3" => self.log_color(
                entity,
                image.data,
                resolution,
                ColorModel::RGB,
                ChannelDatatype::F32,
            )?,
            "32FC1" => self.log_depth(entity, image.data, resolution, ChannelDatatype::F32)?,
            "64FC1" => self.log_depth(entity, image.data, resolution, ChannelDatatype::F64)?,
            "16UC1" => self.log_depth(entity, image.data, resolution, ChannelDatatype::U16)?,
            "16SC1" => self.log_depth(entity, image.data, resolution, ChannelDatatype::I16)?,
            encoding => bail!("unsupported image encoding {encoding}"),
        }
        Ok(image.header.frame_id)
    }

    fn log_color(
        &self,
        entity: &str,
        data: Vec<u8>,
        resolution: [u32; 2],
        model: ColorModel,
        datatype: ChannelDatatype,
    ) -> Result<()> {
        self.recording.log(
            entity,
            &rerun::Image::from_color_model_and_bytes(data, resolution, model, datatype),
        )?;
        Ok(())
    }

    fn log_depth(
        &self,
        entity: &str,
        data: Vec<u8>,
        resolution: [u32; 2],
        datatype: ChannelDatatype,
    ) -> Result<()> {
        self.recording.log(
            entity,
            &rerun::DepthImage::from_data_type_and_bytes(data, resolution, datatype),
        )?;
        Ok(())
    }

    fn log_pointcloud(&self, entity: &str, payload: Vec<u8>) -> Result<String> {
        let cloud = PointCloud2::decode(&payload)?;
        let mut offsets = [usize::MAX; 3];
        for field in &cloud.fields {
            match field.name.as_str() {
                "x" | "y" | "z" => {
                    if field.offset < 0
                        || field.datatype != PointField::FLOAT32 as u8
                        || field.count != 1
                    {
                        bail!("PointCloud2 position fields must be scalar float32 values");
                    }
                    let index = match field.name.as_str() {
                        "x" => 0,
                        "y" => 1,
                        "z" => 2,
                        _ => unreachable!(),
                    };
                    offsets[index] = field.offset as usize;
                }
                _ => {}
            }
        }
        if offsets.contains(&usize::MAX) {
            bail!("PointCloud2 is missing x, y, or z");
        }
        if cloud.width < 0 || cloud.height < 0 || cloud.point_step <= 0 {
            bail!("PointCloud2 dimensions must be nonnegative and point_step must be positive");
        }
        let width = cloud.width as usize;
        let count = width
            .checked_mul(cloud.height as usize)
            .context("PointCloud2 point count overflow")?;
        let step = cloud.point_step as usize;
        if offsets.iter().any(|offset| offset + 4 > step) {
            bail!("PointCloud2 has invalid field offsets or point_step");
        }
        let row_step = width
            .checked_mul(step)
            .context("PointCloud2 row size overflow")?;
        let data_len = count
            .checked_mul(step)
            .context("PointCloud2 data size overflow")?;
        if cloud.row_step as usize != row_step || cloud.data.len() < data_len {
            bail!("PointCloud2 row_step or data length is invalid");
        }
        let mut points = Vec::with_capacity(count);
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for point in cloud.data.chunks_exact(step).take(count) {
            let read = |offset: usize| {
                let bytes: [u8; 4] = point[offset..offset + 4].try_into().unwrap();
                if cloud.is_bigendian {
                    f32::from_be_bytes(bytes)
                } else {
                    f32::from_le_bytes(bytes)
                }
            };
            let position = [read(offsets[0]), read(offsets[1]), read(offsets[2])];
            min_z = min_z.min(position[2]);
            max_z = max_z.max(position[2]);
            points.push(position);
        }
        let range = max_z - min_z + 1.0e-8;
        let class_ids: Vec<u16> = points
            .iter()
            .map(|point| (((point[2] - min_z) / range) * 255.0) as u16)
            .collect();
        self.recording.log(
            entity,
            &rerun::Points3D::new(points)
                .with_radii([0.025])
                .with_class_ids(class_ids),
        )?;
        Ok(cloud.header.frame_id)
    }
}

fn classify(channel: &str) -> Option<(Kind, &str)> {
    for (lcm_suffix, zenoh_suffix, kind) in [
        (IMAGE_LCM_SUFFIX, IMAGE_ZENOH_SUFFIX, Kind::Image),
        (
            POINTCLOUD_LCM_SUFFIX,
            POINTCLOUD_ZENOH_SUFFIX,
            Kind::PointCloud,
        ),
    ] {
        if let Some(topic) = channel.strip_suffix(lcm_suffix) {
            return Some((kind, topic));
        }
        if let Some(topic) = channel.strip_suffix(zenoh_suffix) {
            if let Some(topic) = topic.strip_prefix("dimos") {
                return Some((kind, topic));
            }
        }
    }
    None
}

fn lcm_options(value: &str) -> Result<LcmOptions> {
    let url = url::Url::parse(value)?;
    let multicast_group = url
        .host_str()
        .context("LCM URL has no multicast host")?
        .parse::<Ipv4Addr>()?;
    let mut options = LcmOptions {
        multicast_group,
        port: url.port().context("LCM URL has no port")?,
        ..LcmOptions::default()
    };
    for (name, value) in url.query_pairs() {
        if name == "ttl" {
            options.ttl = value.parse()?;
        }
    }
    Ok(options)
}

async fn listen_lcm(latest: Arc<Latest>, url: &str) -> Result<()> {
    let lcm = Lcm::with_options(lcm_options(url)?).await?;
    loop {
        let packet = lcm.recv().await?;
        if classify(&packet.channel).is_some() {
            latest.push(&packet.channel, packet.data);
        }
    }
}

async fn listen_zenoh(latest: Arc<Latest>, config: &Config) -> Result<()> {
    let mut zconfig = zenoh::Config::default();
    zconfig
        .insert_json5("mode", &serde_json::to_string(&config.zenoh_mode)?)
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    zconfig
        .insert_json5(
            "connect/endpoints",
            &serde_json::to_string(&config.zenoh_connect)?,
        )
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    zconfig
        .insert_json5(
            "listen/endpoints",
            &serde_json::to_string(&config.zenoh_listen)?,
        )
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    let session = zenoh::open(zconfig)
        .await
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    let _subscriber = session
        .declare_subscriber("dimos/**")
        .callback(move |sample| {
            let channel = sample.key_expr().as_str();
            if classify(channel).is_some() {
                latest.push(channel, sample.payload().to_bytes().into_owned());
            }
        })
        .await
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    std::future::pending::<Result<()>>().await
}

async fn process(latest: Arc<Latest>, mut sink: Sink) -> Result<()> {
    loop {
        for (channel, payload) in latest.take().await {
            if let Err(error) = sink.log(&channel, payload) {
                eprintln!("native rerun packet {channel}: {error:#}");
            }
        }
    }
}

async fn run(config: &Config, latest: Arc<Latest>, sink: Sink) -> Result<()> {
    match config.backend.as_str() {
        "lcm" => tokio::try_join!(
            listen_lcm(Arc::clone(&latest), &config.lcm_url),
            process(latest, sink)
        )?,
        "zenoh" => tokio::try_join!(
            listen_zenoh(Arc::clone(&latest), config),
            process(latest, sink)
        )?,
        backend => bail!("unsupported backend {backend}"),
    };
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    let envelope: Envelope = serde_json::from_reader(io::stdin())?;
    let latest = Arc::new(Latest {
        packets: Mutex::new(HashMap::new()),
        notify: Notify::new(),
    });
    let sink = Sink::new(&envelope.config)?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    tokio::select! {
        result = run(&envelope.config, latest, sink) => result?,
        _ = terminate.recv() => {}
    }
    Ok(())
}
