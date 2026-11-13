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

//! Publishes a V4L2 camera as AV1 `CompressedVideo`, encoded with SVT-AV1 in low delay.
//!
//! Two threads, because the work does not fit in one frame time otherwise: the capture thread reads the device
//! and turns each frame into I420 (an MJPEG decode, ~4 ms at 720p), and the encode thread runs SVT-AV1 (~26 ms
//! at 720p30 preset 12 on a Cortex-A76). Between them is a one-frame mailbox where the newest frame wins: for
//! live video a frame that arrived while the encoder was busy is already too old to be worth the queue, and
//! queueing it would add latency to every frame after it. Dropped frames are counted and logged.

use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{native_config, Module, Output};
use lcm_msgs::builtin_interfaces::Time;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use tracing::{info, warn};

use crate::capture::{fourcc, Capture};
use crate::clock::{driver_clocks, wall_s, CaptureClock};
use crate::encoder::{Encoder, Settings};
use crate::jpeg::{is_packed_422, mjpeg, packed_422, I420};

const STATS_EVERY: Duration = Duration::from_secs(10);
const FRAME_TIMEOUT_MS: i32 = 1000;
/// Consecutive frame timeouts before the device is closed and reopened.
const MAX_TIMEOUTS: u32 = 3;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// /dev/videoN, or better a /dev/v4l/by-path link that survives the nodes being renumbered.
    device: String,
    // SVT-AV1 refuses anything under 64 on either side (see encoder::MIN_SIDE).
    #[validate(range(min = 64, max = 16384))]
    width: i64,
    #[validate(range(min = 64, max = 16384))]
    height: i64,
    /// V4L2 pixel format the device is asked for: MJPG, or a packed 4:2:2 format.
    #[validate(length(equal = 4))]
    fourcc: String,
    frame_id: String,
    #[validate(range(min = 1, max = 240))]
    fps: i64,
    #[validate(range(min = 100_000, max = 100_000_000))]
    bitrate_bps: i64,
    /// Frames from one keyframe to the next. 1 would be all-intra, which SVT-AV1 will not encode at a
    /// fixed bitrate.
    #[validate(range(min = 2, max = 600))]
    gop: i64,
    /// SVT-AV1 speed preset: 0 slowest and best, 13 fastest.
    #[validate(range(min = 0, max = 13))]
    preset: i64,
    #[validate(range(min = 1, max = 64))]
    threads: i64,
    /// Seconds between attempts to open a device that is absent or held by another process.
    #[validate(range(min = 0.1, max = 60.0))]
    retry_s: f64,
}

#[derive(Module)]
#[module(name = "av1_camera", setup = start)]
pub struct Av1Camera {
    #[output(encode = CompressedVideo::encode)]
    video: Output<CompressedVideo>,

    #[config]
    config: Config,
}

impl Av1Camera {
    async fn start(&mut self) {
        let config = self.config.clone();
        let output = self.video.clone();
        let runtime = tokio::runtime::Handle::current();
        let mailbox = Arc::new(Mailbox::new());
        let encoder_side = Arc::clone(&mailbox);
        let encode_config = config.clone();
        std::thread::Builder::new()
            .name(format!("av1:{}", config.frame_id))
            .spawn(move || encode(&encode_config, &encoder_side, &output, &runtime))
            .expect("spawn the encode thread");
        std::thread::Builder::new()
            .name(format!("v4l2:{}", config.device))
            .spawn(move || capture_forever(&config, &mailbox))
            .expect("spawn the capture thread");
    }
}

/// One frame, and when it was captured on the wall clock.
struct Stamped {
    frame: I420,
    stamp_s: f64,
}

/// A one-frame handover where the newest frame wins.
pub struct Mailbox {
    slot: Mutex<Option<Stamped>>,
    arrived: Condvar,
    dropped: Mutex<u64>,
}

impl Mailbox {
    fn new() -> Self {
        Self {
            slot: Mutex::new(None),
            arrived: Condvar::new(),
            dropped: Mutex::new(0),
        }
    }

    /// Leaves `frame` for the encoder, discarding whatever had not been taken yet.
    fn put(&self, frame: I420, stamp_s: f64) {
        let mut slot = self.slot.lock().expect("mailbox");
        if slot.is_some() {
            *self.dropped.lock().expect("dropped") += 1;
        }
        *slot = Some(Stamped { frame, stamp_s });
        drop(slot);
        self.arrived.notify_one();
    }

    /// Waits for a frame. `None` when nothing arrived within `timeout`.
    fn take(&self, timeout: Duration) -> Option<Stamped> {
        let mut slot = self.slot.lock().expect("mailbox");
        if slot.is_none() {
            let (next, _) = self
                .arrived
                .wait_timeout(slot, timeout)
                .expect("mailbox wait");
            slot = next;
        }
        slot.take()
    }

    fn take_dropped(&self) -> u64 {
        std::mem::take(&mut *self.dropped.lock().expect("dropped"))
    }
}

/// Opens the device, converts frames and leaves them in the mailbox, reopening whenever it stops delivering.
fn capture_forever(config: &Config, mailbox: &Mailbox) {
    let code = match fourcc(&config.fourcc) {
        Ok(code) => code,
        Err(error) => return warn!(%error, "bad fourcc, not capturing"),
    };
    let compressed = &code.to_le_bytes() == b"MJPG";
    if !compressed && !is_packed_422(code) {
        return warn!(
            fourcc = %config.fourcc,
            "this module takes MJPG or a packed 4:2:2 format, not capturing",
        );
    }
    let mut warned = false;
    loop {
        match Capture::open(
            &config.device,
            config.width as u32,
            config.height as u32,
            code,
            // Nothing here wants the Jetson's JPEG encoder: the frames are going to AV1.
            false,
            0,
        ) {
            Ok(capture) => {
                warned = false;
                info!(device = %config.device, fourcc = %config.fourcc, "streaming");
                pump(config, code, compressed, capture, mailbox);
            }
            Err(error) if !warned => {
                warn!(device = %config.device, %error, "cannot open; is another process holding it? retrying");
                warned = true;
            }
            Err(_) => {}
        }
        std::thread::sleep(Duration::from_secs_f64(config.retry_s));
    }
}

/// Reads and converts until the device stops delivering.
fn pump(config: &Config, code: u32, compressed: bool, mut capture: Capture, mailbox: &Mailbox) {
    let mut clock = CaptureClock::new(driver_clocks(), wall_s);
    let mut decompressor = match turbojpeg::Decompressor::new() {
        Ok(decompressor) => decompressor,
        Err(error) => return warn!(%error, "cannot start libjpeg-turbo; stopping"),
    };
    let (width, height) = (config.width as usize, config.height as usize);
    let mut timeouts = 0;
    let (mut frames, mut convert_s, mut window) = (0u32, 0.0, Instant::now());
    loop {
        let frame = match capture.next(FRAME_TIMEOUT_MS) {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                timeouts += 1;
                if timeouts >= MAX_TIMEOUTS {
                    return warn!(device = %config.device, "stopped delivering frames; reopening");
                }
                continue;
            }
            Err(error) => {
                return warn!(device = %config.device, %error, "capture failed; reopening")
            }
        };
        timeouts = 0;
        let stamp = clock.stamp(frame.driver_stamp_s).unwrap_or_else(wall_s);
        let started = Instant::now();
        let converted = if compressed {
            mjpeg(frame.data, &mut decompressor)
        } else {
            packed_422(frame.data, frame.stride, width, height, code)
        };
        convert_s += started.elapsed().as_secs_f64();
        let picture = match converted {
            Ok(picture) => picture,
            Err(error) => {
                // One corrupt JPEG is not a reason to drop the stream; a format we cannot read is, and that
                // shows up as every frame failing.
                warn!(%error, "could not read this frame; skipped");
                continue;
            }
        };
        if (picture.width, picture.height) != (width, height) {
            return warn!(
                got = format!("{}x{}", picture.width, picture.height),
                asked = format!("{width}x{height}"),
                "the device delivers a different size than the encoder was opened for; reopening",
            );
        }
        mailbox.put(picture, stamp);
        frames += 1;
        if window.elapsed() >= STATS_EVERY {
            let seconds = window.elapsed().as_secs_f64();
            info!(
                frame_id = %config.frame_id,
                fps = f64::from(frames) / seconds,
                convert_ms = convert_s * 1e3 / f64::from(frames),
                "capture stats",
            );
            (frames, convert_s, window) = (0, 0.0, Instant::now());
        }
    }
}

/// Encodes whatever is in the mailbox and publishes it, opening the encoder on the first frame.
fn encode(
    config: &Config,
    mailbox: &Mailbox,
    output: &Output<CompressedVideo>,
    runtime: &tokio::runtime::Handle,
) {
    let settings = Settings {
        width: config.width as u32,
        height: config.height as u32,
        fps: config.fps as u32,
        bitrate_bps: config.bitrate_bps as u32,
        gop: config.gop as u32,
        preset: config.preset as i32,
        threads: config.threads as u32,
    };
    let mut encoder = match Encoder::open(settings) {
        Ok(encoder) => encoder,
        Err(error) => {
            return warn!(%error, "cannot open the AV1 encoder; nothing will be published")
        }
    };
    let header = encoder.stream_header();
    info!(
        width = settings.width,
        height = settings.height,
        fps = settings.fps,
        bitrate_bps = settings.bitrate_bps,
        gop = settings.gop,
        preset = settings.preset,
        threads = settings.threads,
        header_bytes = header.len(),
        "SVT-AV1 open, low delay",
    );
    let mut pts = 0i64;
    // Frames sent and not yet seen coming out, oldest first: (pts, capture stamp).
    let mut waiting: std::collections::VecDeque<(i64, f64)> = std::collections::VecDeque::new();
    let (mut frames, mut keyframes, mut bytes, mut encode_s, mut window) =
        (0u32, 0u32, 0usize, 0.0, Instant::now());
    loop {
        let Some(Stamped { frame, stamp_s }) = mailbox.take(Duration::from_secs(1)) else {
            continue;
        };
        let started = Instant::now();
        let sent = encoder.send(
            &frame.y,
            &frame.u,
            &frame.v,
            frame.width as u32,
            frame.uv_width() as u32,
            pts,
        );
        if let Err(error) = sent {
            warn!(%error, "the encoder refused a frame; stopping");
            return;
        }
        waiting.push_back((pts, stamp_s));
        pts = pts.wrapping_add(1);
        // Exactly one get per send: in low delay SVT-AV1's get_packet BLOCKS until the unit for the frame just
        // sent is ready, so a drain loop would sit waiting for a frame that has not been captured yet.
        {
            let unit = match encoder.next_unit() {
                Ok(Some(unit)) => unit,
                // Nothing ready: the stamp stays queued and pairs with the unit when it comes.
                Ok(None) => {
                    encode_s += started.elapsed().as_secs_f64();
                    continue;
                }
                Err(error) => {
                    warn!(%error, "the encoder failed; stopping");
                    return;
                }
            };
            let (unit, keyframe, unit_pts) = unit;
            // Drop the frames the encoder passed over, so a stamp is never used for a later picture.
            while waiting
                .front()
                .is_some_and(|&(queued, _)| queued < unit_pts)
            {
                waiting.pop_front();
            }
            let stamp = match waiting.front() {
                Some(&(queued, stamp)) if queued == unit_pts => {
                    waiting.pop_front();
                    stamp
                }
                // Should not happen: the unit is for a frame we never sent. Stamp it now rather than drop it.
                _ => wall_s(),
            };
            // A page that subscribes mid-stream has not seen the sequence header, so every keyframe carries one.
            let data = if keyframe && !header.is_empty() {
                let mut with_header = Vec::with_capacity(header.len() + unit.len());
                with_header.extend_from_slice(&header);
                with_header.extend_from_slice(&unit);
                with_header
            } else {
                unit
            };
            frames += 1;
            keyframes += u32::from(keyframe);
            bytes += data.len();
            let message = CompressedVideo {
                timestamp: Time {
                    sec: stamp.trunc() as i32,
                    nanosec: (stamp.fract() * 1e9) as i32,
                },
                frame_id: config.frame_id.clone(),
                data,
                format: "av1".into(),
            };
            runtime.block_on(output.publish(&message)).ok();
        }
        encode_s += started.elapsed().as_secs_f64();
        if window.elapsed() >= STATS_EVERY {
            let seconds = window.elapsed().as_secs_f64();
            info!(
                frame_id = %config.frame_id,
                fps = f64::from(frames) / seconds,
                keyframes_per_s = f64::from(keyframes) / seconds,
                mbit_per_s = bytes as f64 * 8e-6 / seconds,
                encode_ms = encode_s * 1e3 / f64::from(frames.max(1)),
                dropped = mailbox.take_dropped(),
                "AV1 encode stats",
            );
            (frames, keyframes, bytes, encode_s, window) = (0, 0, 0, 0.0, Instant::now());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(value: u8) -> I420 {
        let mut picture = I420::grey(8, 8);
        picture.y.fill(value);
        picture
    }

    #[test]
    fn the_mailbox_hands_over_the_frame_that_was_left_in_it() {
        let mailbox = Mailbox::new();
        mailbox.put(frame(7), 1.5);
        let taken = mailbox.take(Duration::from_millis(10)).expect("a frame");
        assert_eq!(taken.frame.y[0], 7);
        assert_eq!(taken.stamp_s, 1.5);
        assert_eq!(mailbox.take_dropped(), 0);
    }

    #[test]
    fn the_newest_frame_wins_and_the_overtaken_one_is_counted() {
        let mailbox = Mailbox::new();
        mailbox.put(frame(1), 1.0);
        mailbox.put(frame(2), 2.0);
        mailbox.put(frame(3), 3.0);
        let taken = mailbox.take(Duration::from_millis(10)).expect("a frame");
        assert_eq!(taken.frame.y[0], 3, "the encoder gets the newest frame");
        assert_eq!(mailbox.take_dropped(), 2);
        assert_eq!(mailbox.take_dropped(), 0, "the count is cleared when read");
    }

    #[test]
    fn an_empty_mailbox_times_out_instead_of_blocking_forever() {
        let mailbox = Mailbox::new();
        let started = Instant::now();
        assert!(mailbox.take(Duration::from_millis(20)).is_none());
        assert!(started.elapsed() >= Duration::from_millis(20));
    }

    #[test]
    fn a_frame_left_while_the_encoder_waits_wakes_it() {
        let mailbox = Arc::new(Mailbox::new());
        let writer = Arc::clone(&mailbox);
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            writer.put(frame(9), 4.0);
        });
        let taken = mailbox.take(Duration::from_secs(2)).expect("woken");
        assert_eq!(taken.frame.y[0], 9);
        handle.join().expect("writer");
    }
}
