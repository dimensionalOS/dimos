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

use crate::session::{Hub, Session};

/// Bulk chunk size — safely under the 64 KB SCTP message-size interop floor.
pub const CHUNK: usize = 60_000;
/// Pose sends are dropped (not queued) beyond this buffered amount — freshness over completeness.
const POSE_BUF_MAX: usize = 64_000;
/// Bulk waits below this before each chunk, so the backlog accumulates in the outbox (where
/// conflation keeps it fresh), not in the SCTP send buffer.
const BULK_BUF_HIGH: usize = 512_000;

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
    info!(port, "WebRTC listening (udp, muxed)");
    while let Some((rsid, sdp)) = offers.recv().await {
        let hub = hub.clone();
        let api = api.clone();
        tokio::spawn(async move {
            match answer(&hub, &api, sdp).await {
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

async fn build_api(port: u16, public_ip: Option<String>) -> Result<API> {
    // One UDP socket for every session (ICE mux). Dual-stack [::] for IPv6-first hosts.
    let socket = tokio::net::UdpSocket::bind(("::", port))
        .await
        .with_context(|| format!("bind udp :{port}"))?;
    let mut se = SettingEngine::default();
    se.set_udp_network(UDPNetwork::Muxed(UDPMuxDefault::new(UDPMuxParams::new(
        socket,
    ))));
    if let Some(ip) = public_ip {
        se.set_nat_1to1_ips(vec![ip], RTCIceCandidateType::Host);
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
async fn answer(hub: &Arc<Hub>, api: &API, offer_sdp: String) -> Result<String> {
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
                        let t = tokio::spawn(drain_pose(dc, sess));
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

/// Datagram semantics: never wait on the SCTP buffer — a full buffer drops the frame, so a stalled
/// association can't turn pose into a queue of stale samples.
async fn drain_pose(dc: Arc<RTCDataChannel>, sess: Arc<Session>) {
    loop {
        let frame = sess.dgram.get().await;
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
}
