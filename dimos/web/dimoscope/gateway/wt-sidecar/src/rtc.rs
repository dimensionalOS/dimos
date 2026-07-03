//! WebRTC DataChannel plane — the same Hub/Session/outbox machinery as WebTransport, mapped onto
//! SCTP's primitives so the transports compare fairly. The browser client is
//! packages/web/src/transports/experimental/webRtcData.ts; signaling rides the gateway's /rtc
//! websocket and reaches us as rtc-offer{rsid,sdp} over the pipe (answered with rtc-answer).
//!
//!   ctl   ordered+reliable    one JSON per message: the full control protocol — subscribe/QoS/list/
//!                             ping AND teleop/goal/rpc (forwarded over the pipe to SafetyEgress)
//!   pose  unordered+lossy     [f64 send-ms][LC02] frames ≤ DATAGRAM_MAX, one per message; dropped
//!                             (not queued) when the SCTP buffer is full — datagram semantics
//!   bulk  ordered+reliable    [u32be len][frame] in ≤CHUNK messages (SCTP can't carry a 1 MB
//!                             message); the client length-prefix-parses the concatenation
//!
//! All sessions share ONE muxed UDP socket (:RTC_PORT) — one firewall rule, and port-scoped tooling
//! (netem) sees it. No STUN server-side: the host's own IP is a host candidate (a server needs a
//! reachable address to serve at all); set RTC_PUBLIC_IP when the host is NATed.

use std::sync::Arc;

use anyhow::{Context, Result};
use bytes::{BufMut, Bytes, BytesMut};
use serde_json::{json, Value};
use tokio::sync::{mpsc, Notify};
use tracing::{info, warn};
use webrtc::api::interceptor_registry::register_default_interceptors;
use webrtc::api::media_engine::MediaEngine;
use webrtc::api::setting_engine::SettingEngine;
use webrtc::api::{APIBuilder, API};
use webrtc::data_channel::RTCDataChannel;
use webrtc::ice::udp_mux::{UDPMuxDefault, UDPMuxParams};
use webrtc::ice::udp_network::UDPNetwork;
use webrtc::ice_transport::ice_candidate_type::RTCIceCandidateType;
use webrtc::interceptor::registry::Registry;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::peer_connection_state::RTCPeerConnectionState;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;

use crate::session::{now_ms, Hub, Session};

/// Bulk chunk size — safely under the 64 KB SCTP message-size interop floor.
pub const CHUNK: usize = 60_000;
/// Pose sends are dropped (not queued) beyond this buffered amount — freshness over completeness.
const POSE_BUF_MAX: usize = 64_000;
/// Default pose-age TTL (ms): drop a pose frame already older than this at drain time, so a cleared
/// stall delivers newest-wins instead of a stale burst — the WT datagram drain's WT_DGRAM_TTL_MS,
/// mirrored here (the pose channel gates only on buffer *size*, POSE_BUF_MAX, without it). Pose
/// frames are the same [f64be ingress-ms][LC02] the pipe stamps, so the age check is identical.
/// RTC_POSE_TTL_MS overrides; 0 = off.
const POSE_TTL_MS_DEFAULT: f64 = 200.0;
/// Bulk waits below this before each chunk, so the backlog accumulates in the outbox (where
/// conflation keeps it fresh), not in the SCTP send buffer. Kept small ON PURPOSE: pose shares
/// the association and webrtc-rs has no cross-stream scheduler (RFC 8260), so every queued bulk
/// byte transmits ahead of pose frames. At 512 KB this queue was ~500 ms of wire time on wifi
/// rates — the pose channel's buffer never drained, its drop gate fired on nearly every frame
/// (fast lane ×5.6 late, 2% delivered beside a 2 MB/s flood). ~1.5 chunks bounds the bulk that
/// can sit ahead of pose; the buffered-amount-low refill keeps bulk itself saturated.
const BULK_BUF_HIGH: usize = 96_000;

/// `[u32be len][frame]` — the exact framing the browser's bulk reassembler parses.
pub fn bulk_framed(frame: &Bytes) -> Bytes {
    let mut buf = BytesMut::with_capacity(4 + frame.len());
    buf.put_u32(frame.len() as u32);
    buf.put_slice(frame);
    buf.freeze()
}

/// Run forever: build one shared API (muxed UDP socket) and answer every relayed offer.
pub async fn run(
    hub: Arc<Hub>,
    port: u16,
    public_ip: Option<String>,
    mut offers: mpsc::UnboundedReceiver<(u64, String)>,
) {
    let api = match build_api(port, public_ip).await {
        Ok(api) => Arc::new(api),
        Err(e) => {
            warn!("webrtc plane disabled — mux socket failed: {e:#}");
            return;
        }
    };
    // Read once at plane start (env, not const, so it's tunable without touching main.rs). f64 is
    // Copy, so every session gets it for free through answer → drain_pose.
    let pose_ttl_ms: f64 = std::env::var("RTC_POSE_TTL_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(POSE_TTL_MS_DEFAULT);
    info!(port, pose_ttl_ms, "WebRTC listening (udp, muxed)");
    while let Some((rsid, sdp)) = offers.recv().await {
        let hub = hub.clone();
        let api = api.clone();
        tokio::spawn(async move {
            match answer(&hub, &api, sdp, pose_ttl_ms).await {
                Ok(answer_sdp) => {
                    hub.send_upstream(json!({"op": "rtc-answer", "rsid": rsid, "sdp": answer_sdp}))
                }
                Err(e) => {
                    warn!(rsid, "webrtc offer failed: {e:#}");
                    hub.send_upstream(
                        json!({"op": "rtc-answer", "rsid": rsid, "error": format!("{e:#}")}),
                    );
                }
            }
        });
    }
}

/// The IPv4 the default route uses (connect() picks the route; no packet is sent).
fn primary_local_v4() -> Result<std::net::IpAddr> {
    let s = std::net::UdpSocket::bind("0.0.0.0:0")?;
    s.connect("8.8.8.8:80")?;
    Ok(s.local_addr()?.ip())
}

async fn build_api(port: u16, public_ip: Option<String>) -> Result<API> {
    // One UDP socket for every session (ICE mux). v4-only ON PURPOSE: a dual-stack [::] bind
    // reports v4 peers as ::ffff:a.b.c.d, and webrtc-ice fails to match that form against the
    // browser's plain-v4 candidates ("discard success message … no such remote" — ICE never
    // completes). WebRTC dials literal candidate IPs (never names), and we only advertise v4,
    // so nothing is lost. (WebTransport keeps its dual-stack bind: browsers resolve localhost
    // to ::1 first.)
    let socket = tokio::net::UdpSocket::bind(("0.0.0.0", port))
        .await
        .with_context(|| format!("bind udp :{port}"))?;
    let mut se = SettingEngine::default();
    se.set_udp_network(UDPNetwork::Muxed(UDPMuxDefault::new(UDPMuxParams::new(
        socket,
    ))));
    if let Some(ip) = public_ip {
        se.set_nat_1to1_ips(vec![ip], RTCIceCandidateType::Host);
        // The 1:1 mapper rewrites EVERY local IP to the public one, so a multi-NIC host (EC2 ENI +
        // a VPN interface) yields identical candidates — and webrtc-ice closes the duplicate, which
        // closes the SHARED mux conn and kills the session ("udp_mux: … buffer: closed"). Keep
        // exactly one local IP; which one is irrelevant (the SDP carries the public IP and the mux
        // socket listens on ::).
        let primary = primary_local_v4().context("primary local v4 for RTC_PUBLIC_IP")?;
        info!(%primary, "webrtc 1:1 NAT: mapping only the primary interface");
        se.set_ip_filter(Box::new(move |a: std::net::IpAddr| a == primary));
    }
    let mut media = MediaEngine::default();
    media.register_default_codecs()?;
    let registry = register_default_interceptors(Registry::new(), &mut media)?;
    Ok(APIBuilder::new()
        .with_media_engine(media)
        .with_interceptor_registry(registry)
        .with_setting_engine(se)
        .build())
}

/// One browser session: peer connection + Hub session + the three channels' tasks.
async fn answer(hub: &Arc<Hub>, api: &API, offer_sdp: String, pose_ttl_ms: f64) -> Result<String> {
    let pc = Arc::new(api.new_peer_connection(RTCConfiguration::default()).await?);

    let (ctl_tx, ctl_rx) = mpsc::unbounded_channel();
    let sess = hub.add_session(ctl_tx, "rtc-rs");
    let sid = sess.sid;
    // The ctl-forward task claims this receiver when the ctl channel opens.
    let ctl_rx = Arc::new(std::sync::Mutex::new(Some(ctl_rx)));
    // Channel tasks register here; the state-change handler aborts them (an aborted drain would
    // otherwise pend forever on an outbox that no longer receives frames).
    let tasks: Arc<std::sync::Mutex<Vec<tokio::task::JoinHandle<()>>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));

    {
        let hub = hub.clone();
        let sess = sess.clone();
        let tasks = tasks.clone();
        let ctl_rx = ctl_rx.clone();
        pc.on_data_channel(Box::new(move |dc: Arc<RTCDataChannel>| {
            let hub = hub.clone();
            let sess = sess.clone();
            let tasks = tasks.clone();
            let ctl_rx = ctl_rx.clone();
            Box::pin(async move {
                match dc.label() {
                    "ctl" => wire_ctl(hub, sess, dc, ctl_rx, tasks).await,
                    "pose" => {
                        let t = tokio::spawn(drain_pose(dc, sess, pose_ttl_ms));
                        tasks.lock().expect("tasks lock").push(t);
                    }
                    "bulk" => {
                        let t = tokio::spawn(drain_bulk(dc, sess));
                        tasks.lock().expect("tasks lock").push(t);
                    }
                    other => warn!(sid, "unexpected datachannel label {other:?}"),
                }
            })
        }));
    }

    {
        let hub = hub.clone();
        let tasks = tasks.clone();
        let pc2 = pc.clone();
        pc.on_peer_connection_state_change(Box::new(move |st: RTCPeerConnectionState| {
            let hub = hub.clone();
            let tasks = tasks.clone();
            let pc2 = pc2.clone();
            Box::pin(async move {
                if matches!(
                    st,
                    RTCPeerConnectionState::Failed | RTCPeerConnectionState::Closed
                ) {
                    info!(sid, ?st, "webrtc session closed");
                    for t in tasks.lock().expect("tasks lock").drain(..) {
                        t.abort();
                    }
                    hub.remove_session(sid); // → disconnect{sid} upstream → deadman cancel
                    let _ = pc2.close().await;
                }
            })
        }));
    }

    // Non-trickle: gather every candidate, then hand back one complete SDP.
    pc.set_remote_description(RTCSessionDescription::offer(offer_sdp)?)
        .await?;
    let answer = pc.create_answer(None).await?;
    let mut gathered = pc.gathering_complete_promise().await;
    pc.set_local_description(answer).await?;
    let _ = gathered.recv().await;
    let local = pc
        .local_description()
        .await
        .context("no local description after gathering")?;
    info!(sid, "webrtc session answered");
    Ok(local.sdp)
}

/// ctl: browser ops in (the full control protocol — session.on_control forwards the write path to
/// the gateway), hub replies out. Hello is pushed on open (the WT client pulls it with "list").
async fn wire_ctl(
    hub: Arc<Hub>,
    sess: Arc<Session>,
    dc: Arc<RTCDataChannel>,
    ctl_rx: Arc<std::sync::Mutex<Option<mpsc::UnboundedReceiver<Value>>>>,
    tasks: Arc<std::sync::Mutex<Vec<tokio::task::JoinHandle<()>>>>,
) {
    {
        let hub = hub.clone();
        let sess = sess.clone();
        dc.on_message(Box::new(move |m| {
            let hub = hub.clone();
            let sess = sess.clone();
            Box::pin(async move {
                // bad control JSON is dropped, never fatal to the session
                if let Ok(v) = serde_json::from_slice::<Value>(&m.data) {
                    sess.on_control(&hub, &v);
                }
            })
        }));
    }
    let dc2 = dc.clone();
    dc.on_open(Box::new(move || {
        Box::pin(async move {
            let _ = dc2.send_text(sess.hello(&hub).to_string()).await;
            let Some(mut rx) = ctl_rx.lock().expect("ctl_rx lock").take() else {
                return;
            };
            let t = tokio::spawn(async move {
                while let Some(v) = rx.recv().await {
                    if dc2.send_text(v.to_string()).await.is_err() {
                        return;
                    }
                }
            });
            tasks.lock().expect("tasks lock").push(t);
        })
    }));
}

/// True if a pose frame ([f64be ingress-ms] prefix) has aged past `ttl_ms` by `now` (both ms, same
/// wall clock as `now_ms`). `ttl_ms <= 0` disables the gate; a frame too short to carry a stamp is
/// never expired (fail-open — better to send an unstamped frame than drop it).
fn pose_expired(frame: &[u8], ttl_ms: f64, now: f64) -> bool {
    if ttl_ms <= 0.0 || frame.len() < 8 {
        return false;
    }
    let stamp = f64::from_be_bytes(frame[..8].try_into().expect("8 bytes"));
    now - stamp > ttl_ms
}

/// Datagram semantics: never wait on the SCTP buffer — a full buffer drops the frame, so a stalled
/// association can't turn pose into a queue of stale samples. The age gate (mirroring the WT
/// datagram drain) drops a frame that aged past the TTL while queued, so a cleared stall delivers
/// the newest pose instead of a stale burst the buffer-size gate alone would let through.
async fn drain_pose(dc: Arc<RTCDataChannel>, sess: Arc<Session>, ttl_ms: f64) {
    loop {
        let frame = sess.dgram.get().await;
        if pose_expired(&frame, ttl_ms, now_ms()) {
            continue; // drop: aged past the TTL waiting to drain
        }
        if dc.buffered_amount().await > POSE_BUF_MAX {
            continue; // drop: freshness over completeness
        }
        if dc.send(&frame).await.is_err() {
            return;
        }
    }
}

/// Reliable lane with real backpressure: wait below BULK_BUF_HIGH before each chunk so the backlog
/// accumulates in the outbox (WRR + conflation keep it fresh), not in the SCTP send buffer.
async fn drain_bulk(dc: Arc<RTCDataChannel>, sess: Arc<Session>) {
    let low = Arc::new(Notify::new());
    dc.set_buffered_amount_low_threshold(BULK_BUF_HIGH / 2)
        .await;
    {
        let low = low.clone();
        dc.on_buffered_amount_low(Box::new(move || {
            let low = low.clone();
            Box::pin(async move {
                low.notify_one();
            })
        }))
        .await;
    }
    loop {
        let frame = sess.bulk.get().await;
        let framed = bulk_framed(&frame);
        let mut off = 0;
        while off < framed.len() {
            while dc.buffered_amount().await > BULK_BUF_HIGH {
                low.notified().await;
            }
            let end = (off + CHUNK).min(framed.len());
            if dc.send(&framed.slice(off..end)).await.is_err() {
                return;
            }
            off = end;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bulk_framing_round_trips_across_chunks() {
        let frame = Bytes::from((0..180_000u32).map(|i| i as u8).collect::<Vec<_>>());
        let framed = bulk_framed(&frame);
        // chunk exactly as drain_bulk does, then reassemble as the browser does
        let chunks: Vec<Bytes> = (0..framed.len())
            .step_by(CHUNK)
            .map(|off| framed.slice(off..(off + CHUNK).min(framed.len())))
            .collect();
        assert!(chunks.len() > 1 && chunks.iter().all(|c| c.len() <= CHUNK));
        let stream: Vec<u8> = chunks.concat();
        let len = u32::from_be_bytes(stream[0..4].try_into().unwrap()) as usize;
        assert_eq!(len, frame.len());
        assert_eq!(&stream[4..4 + len], &frame[..]);
        assert_eq!(stream.len(), 4 + len); // no trailing bytes
    }

    #[test]
    fn pose_expired_gates_on_age() {
        // frame = [f64be ingress-ms=1000][payload]
        let mut f = 1000.0f64.to_be_bytes().to_vec();
        f.extend_from_slice(b"LC02payload");
        assert!(!pose_expired(&f, 200.0, 1150.0)); // age 150 ≤ 200 → fresh
        assert!(pose_expired(&f, 200.0, 1300.0)); // age 300 > 200 → expired
        assert!(!pose_expired(&f, 0.0, 9999.0)); // ttl 0 disables the gate
        assert!(!pose_expired(&[1, 2, 3], 200.0, 9999.0)); // too short to stamp → never expired
    }
}
