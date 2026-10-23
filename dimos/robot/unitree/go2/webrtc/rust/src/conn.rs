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

//! One WebRTC session with a Go2, the one the Unitree app opens: the signaling handshake, a data channel for
//! validation, heartbeat, subscriptions and requests, and the receive-only H.264 camera track. A request waits for
//! its answer.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rand::Rng;
use serde_json::{json, Value};
use tokio::sync::oneshot;
use tracing::{debug, info, warn};
use webrtc::api::interceptor_registry::{configure_nack, configure_rtcp_reports};
use webrtc::api::media_engine::MediaEngine;
use webrtc::api::setting_engine::SettingEngine;
use webrtc::api::APIBuilder;
use webrtc::data_channel::data_channel_message::DataChannelMessage;
use webrtc::data_channel::RTCDataChannel;
use webrtc::interceptor::registry::Registry;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::peer_connection_state::RTCPeerConnectionState;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::peer_connection::RTCPeerConnection;
use webrtc::rtcp::payload_feedbacks::picture_loss_indication::PictureLossIndication;
use webrtc::rtp_transceiver::rtp_codec::RTPCodecType;
use webrtc::rtp_transceiver::rtp_transceiver_direction::RTCRtpTransceiverDirection;
use webrtc::rtp_transceiver::RTCRtpTransceiverInit;

use crate::clock::now_ns;
use crate::lidar::parse_binary;
use crate::signaling::{exchange_sdp, offer_id, validation_reply};
use crate::timing::{secs, Timing};

/// What a live session reports, each with its host arrival time (ns).
pub enum Event {
    /// a message on a subscribed topic: its JSON, and the bytes binary messages carry (the lidar's voxels)
    Data {
        topic: String,
        json: Value,
        binary: Option<Vec<u8>>,
        arrived_ns: u64,
    },
    /// a camera RTP packet (H.264)
    Rtp {
        packet: webrtc::rtp::packet::Packet,
        arrived_ns: u64,
    },
    /// the peer link died (robot off Wi-Fi, out of range, rebooted, closed it)
    Lost,
}

pub type OnEvent = Arc<dyn Fn(Event) + Send + Sync>;

pub struct Conn {
    pc: Arc<RTCPeerConnection>,
    channel: Arc<RTCDataChannel>,
    video_ssrc: Arc<AtomicU64>,
    closed: Arc<AtomicBool>,
    /// requests waiting for the robot's answer, by request id
    pending: Arc<Mutex<HashMap<u64, oneshot::Sender<Value>>>>,
    next_id: AtomicU64,
}

async fn send(
    channel: &RTCDataChannel,
    kind: &str,
    topic: &str,
    data: Option<Value>,
) -> Result<(), String> {
    let mut message = json!({ "type": kind, "topic": topic });
    if let Some(data) = data {
        message["data"] = data;
    }
    let text = message.to_string();
    debug!("dc -> {text}");
    channel
        .send_text(text)
        .await
        .map(|_| ())
        .map_err(|e| format!("data channel send: {e}"))
}

fn robot_api(timing: &Timing) -> Result<webrtc::api::API, String> {
    let mut media = MediaEngine::default();
    media.register_default_codecs().map_err(|e| e.to_string())?;
    // no transport-cc: the robot rejects offers that ask for it
    let mut registry = Registry::new();
    registry = configure_nack(registry, &mut media);
    registry = configure_rtcp_reports(registry);
    // the link is lost after link_lost_timeout of silence
    let mut settings = SettingEngine::default();
    settings.set_ice_timeouts(
        Some(secs(timing.link_quiet_timeout)),
        Some(secs(timing.link_lost_timeout)),
        Some(secs(timing.ice_keepalive_period)),
    );
    Ok(APIBuilder::new()
        .with_setting_engine(settings)
        .with_media_engine(media)
        .with_interceptor_registry(registry)
        .build())
}

/// The id a request carries in `header.identity.id` (also how its answer names it).
fn answer_id(message: &Value) -> Option<u64> {
    message["data"]["header"]["identity"]["id"].as_u64()
}

impl Conn {
    /// Opens the session; returns once the robot has validated the data channel and the camera is asked for.
    pub async fn connect(
        ip: &str,
        aes_key: &str,
        timing: &Timing,
        on_event: OnEvent,
    ) -> Result<Arc<Conn>, String> {
        let pc = Arc::new(
            robot_api(timing)?
                .new_peer_connection(RTCConfiguration::default())
                .await
                .map_err(|e| e.to_string())?,
        );
        let closed = Arc::new(AtomicBool::new(false));
        let result =
            Self::negotiate(pc.clone(), ip, aes_key, timing, on_event, closed.clone()).await;
        if result.is_err() {
            // closing a link that never came up isn't a lost link
            closed.store(true, Ordering::Relaxed);
            let _ = pc.close().await;
        }
        result
    }

    async fn negotiate(
        pc: Arc<RTCPeerConnection>,
        ip: &str,
        aes_key: &str,
        timing: &Timing,
        on_event: OnEvent,
        closed: Arc<AtomicBool>,
    ) -> Result<Arc<Conn>, String> {
        pc.add_transceiver_from_kind(
            RTPCodecType::Video,
            Some(RTCRtpTransceiverInit {
                direction: RTCRtpTransceiverDirection::Recvonly,
                send_encodings: vec![],
            }),
        )
        .await
        .map_err(|e| e.to_string())?;
        let channel = pc
            .create_data_channel("data", None)
            .await
            .map_err(|e| e.to_string())?;
        let video_ssrc = Arc::new(AtomicU64::new(u64::MAX));
        let pending: Arc<Mutex<HashMap<u64, oneshot::Sender<Value>>>> = Default::default();

        // validation: the robot sends a key, we answer md5("UnitreeGo2_" + key); "Validation Ok." means ready
        let (validated_tx, validated_rx) = oneshot::channel::<()>();
        let validated_tx = Arc::new(Mutex::new(Some(validated_tx)));
        let last_key = Arc::new(Mutex::new(String::new()));
        {
            let on_event = on_event.clone();
            let reply_channel = channel.clone();
            let pending = pending.clone();
            channel.on_message(Box::new(move |message: DataChannelMessage| {
                let arrived_ns = now_ns();
                let channel = reply_channel.clone();
                let validated_tx = validated_tx.clone();
                let last_key = last_key.clone();
                let on_event = on_event.clone();
                let pending = pending.clone();
                Box::pin(async move {
                    if !message.is_string {
                        match parse_binary(&message.data) {
                            Some((json, binary)) => {
                                let topic = json["topic"].as_str().unwrap_or("").to_string();
                                on_event(Event::Data {
                                    topic,
                                    json,
                                    binary: Some(binary.to_vec()),
                                    arrived_ns,
                                });
                            }
                            None => warn!(
                                bytes = message.data.len(),
                                "unparsed binary data-channel message"
                            ),
                        }
                        return;
                    }
                    let Ok(message) = serde_json::from_slice::<Value>(&message.data) else {
                        return;
                    };
                    let kind = message["type"].as_str().unwrap_or("");
                    match kind {
                        "validation" => {
                            let data = message["data"].as_str().unwrap_or("").to_string();
                            if data == "Validation Ok." {
                                if let Some(tx) = validated_tx.lock().unwrap().take() {
                                    let _ = tx.send(());
                                }
                            } else {
                                *last_key.lock().unwrap() = data.clone();
                                let _ = send(
                                    &channel,
                                    "validation",
                                    "",
                                    Some(json!(validation_reply(&data))),
                                )
                                .await;
                            }
                        }
                        "err" if message["info"] == "Validation Needed." => {
                            let key = last_key.lock().unwrap().clone();
                            let _ = send(
                                &channel,
                                "validation",
                                "",
                                Some(json!(validation_reply(&key))),
                            )
                            .await;
                        }
                        "msg" => {
                            let topic = message["topic"].as_str().unwrap_or("").to_string();
                            on_event(Event::Data {
                                topic,
                                json: message,
                                binary: None,
                                arrived_ns,
                            });
                        }
                        _ => {
                            let waiting = answer_id(&message)
                                .and_then(|id| pending.lock().unwrap().remove(&id));
                            match waiting {
                                Some(tx) => {
                                    let _ = tx.send(message);
                                }
                                None => debug!("dc <- {message}"),
                            }
                        }
                    }
                })
            }));
        }

        // camera: every RTP packet as it arrives
        {
            let on_event = on_event.clone();
            let closed = closed.clone();
            let video_ssrc = video_ssrc.clone();
            pc.on_track(Box::new(move |track, receiver, _transceiver| {
                let on_event = on_event.clone();
                let closed = closed.clone();
                let video_ssrc = video_ssrc.clone();
                Box::pin(async move {
                    if track.kind() != RTPCodecType::Video {
                        return;
                    }
                    let codec = track.codec();
                    let clock_hz = codec.capability.clock_rate;
                    info!(mime = %codec.capability.mime_type, clock_hz, ssrc = track.ssrc(), "camera track from the Go2");
                    video_ssrc.store(track.ssrc() as u64, Ordering::Relaxed);
                    {
                        let closed = closed.clone();
                        // read so the interceptors (NACK, receiver reports) run; the sender reports' RTP clock isn't the
                        // one the Go2 stamps its packets with (they step a fixed 60 per frame), so they don't place frames
                        tokio::spawn(async move {
                            while receiver.read_rtcp().await.is_ok() {
                                if closed.load(Ordering::Relaxed) {
                                    break;
                                }
                            }
                        });
                    }
                    tokio::spawn(async move {
                        while let Ok((packet, _)) = track.read_rtp().await {
                            if closed.load(Ordering::Relaxed) {
                                break;
                            }
                            on_event(Event::Rtp { packet, arrived_ns: now_ns() });
                        }
                    });
                })
            }));
        }
        {
            let on_event = on_event.clone();
            let closed = closed.clone();
            pc.on_peer_connection_state_change(Box::new(move |state| {
                info!(%state, "Go2 peer connection");
                // Disconnected can recover; Failed is link_lost_timeout of silence
                if matches!(
                    state,
                    RTCPeerConnectionState::Failed | RTCPeerConnectionState::Closed
                ) && !closed.swap(true, Ordering::Relaxed)
                {
                    on_event(Event::Lost);
                }
                Box::pin(async {})
            }));
        }

        let offer = pc.create_offer(None).await.map_err(|e| e.to_string())?;
        let mut gathered = pc.gathering_complete_promise().await;
        pc.set_local_description(offer)
            .await
            .map_err(|e| e.to_string())?;
        let _ = tokio::time::timeout(secs(timing.ice_gather_timeout), gathered.recv()).await;
        let local = pc.local_description().await.ok_or("no local description")?;
        let offer_json =
            json!({ "id": offer_id(ip), "sdp": local.sdp, "type": "offer", "token": "" })
                .to_string();
        let sdp = exchange_sdp(ip, aes_key, &offer_json, secs(timing.signaling_timeout)).await?;
        let answer = RTCSessionDescription::answer(sdp).map_err(|e| e.to_string())?;
        pc.set_remote_description(answer)
            .await
            .map_err(|e| e.to_string())?;

        let validation_timeout = secs(timing.validation_timeout);
        match tokio::time::timeout(validation_timeout, validated_rx).await {
            Ok(Ok(())) => info!("Go2 validated the data channel"),
            _ => {
                return Err(format!(
                    "the Go2 never validated the data channel within {validation_timeout:?}"
                ))
            }
        }
        let conn = Arc::new(Conn {
            pc,
            channel,
            video_ssrc,
            closed,
            pending,
            // ids are the robot's i32: start anywhere, then count
            next_id: AtomicU64::new(rand::thread_rng().gen_range(0..1 << 30)),
        });
        conn.start_heartbeat(secs(timing.heartbeat_period));
        Ok(conn)
    }

    fn start_heartbeat(self: &Arc<Self>, period: Duration) {
        let conn = Arc::downgrade(self);
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(period).await;
                let Some(conn) = conn.upgrade() else { break };
                if conn.closed.load(Ordering::Relaxed) {
                    break;
                }
                let now = now_ns() / 1_000_000_000;
                let data = json!({ "timeInStr": format_utc(now), "timeInNum": now });
                let _ = send(&conn.channel, "heartbeat", "", Some(data)).await;
            }
        });
    }

    pub fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Relaxed)
    }

    /// A `type: msg` publish that expects no answer (the wireless controller, the lidar switch).
    pub async fn publish(&self, topic: &str, data: Value) -> Result<(), String> {
        send(&self.channel, "msg", topic, Some(data)).await
    }

    /// A `type: req` that waits for no answer (a stream of Move commands).
    pub async fn send_req(&self, topic: &str, payload: Value) -> Result<(), String> {
        send(&self.channel, "req", topic, Some(payload)).await
    }

    pub async fn send_raw(&self, kind: &str, data: Value) -> Result<(), String> {
        send(&self.channel, kind, "", Some(data)).await
    }

    /// A `type: req` on `topic`, like `publish_request_new(topic, {"api_id", "parameter"})`; a string parameter is
    /// sent as it is, anything else as its JSON text. Waits up to `timeout` for the robot's answer (its message).
    pub async fn request(
        &self,
        topic: &str,
        api_id: i64,
        parameter: Option<&Value>,
        timeout: Duration,
    ) -> Result<Value, String> {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed) % (1 << 31);
        let parameter = match parameter {
            None => json!(""),
            Some(Value::String(text)) => json!(text),
            Some(value) => json!(value.to_string()),
        };
        let (tx, rx) = oneshot::channel();
        self.pending.lock().unwrap().insert(id, tx);
        let payload = json!({ "header": { "identity": { "id": id, "api_id": api_id } }, "parameter": parameter });
        if let Err(err) = send(&self.channel, "req", topic, Some(payload)).await {
            self.pending.lock().unwrap().remove(&id);
            return Err(err);
        }
        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(answer)) => Ok(answer),
            _ => {
                self.pending.lock().unwrap().remove(&id);
                Err(format!(
                    "no answer from the Go2 to {topic} api {api_id} within {timeout:?}"
                ))
            }
        }
    }

    pub async fn subscribe(&self, topic: &str) -> Result<(), String> {
        send(&self.channel, "subscribe", topic, None).await
    }

    /// Ask the robot for a fresh keyframe (start, or after lost packets).
    pub async fn request_keyframe(&self) {
        let ssrc = self.video_ssrc.load(Ordering::Relaxed);
        if ssrc != u64::MAX {
            let _ = self
                .pc
                .write_rtcp(&[Box::new(PictureLossIndication {
                    sender_ssrc: 0,
                    media_ssrc: ssrc as u32,
                })])
                .await;
        }
    }

    pub async fn close(&self) {
        self.closed.store(true, Ordering::Relaxed);
        let _ = self.pc.close().await;
    }
}

/// "YYYY-MM-DD HH:MM:SS" (UTC) for the heartbeat, without a date crate.
pub fn format_utc(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // civil-from-days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + if month <= 2 { 1 } else { 0 };
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02}",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_format() {
        assert_eq!(format_utc(0), "1970-01-01 00:00:00");
        assert_eq!(format_utc(1_790_000_000), "2026-09-21 14:13:20");
    }

    #[tokio::test]
    async fn robot_offer_has_no_transport_cc() {
        let pc = robot_api(&Timing::default())
            .unwrap()
            .new_peer_connection(RTCConfiguration::default())
            .await
            .unwrap();
        pc.add_transceiver_from_kind(
            RTPCodecType::Video,
            Some(RTCRtpTransceiverInit {
                direction: RTCRtpTransceiverDirection::Recvonly,
                send_encodings: vec![],
            }),
        )
        .await
        .unwrap();
        pc.create_data_channel("data", None).await.unwrap();
        let offer = pc.create_offer(None).await.unwrap().sdp;
        assert!(offer.contains("H264") && offer.contains("webrtc-datachannel"));
        assert!(
            !offer.contains("transport-cc") && !offer.contains("transport-wide-cc"),
            "{offer}"
        );
        pc.close().await.unwrap();
    }
}
