//! Pipe client: the sidecar's feed from the Python gateway (gateway/pipe.py) over a unix socket.
//! Reconnects with backoff; the gateway listens. Framing [u32be len][u8 kind][payload]:
//! kind 1 = DATA (raw LC02 packet — topic/type live inside the channel), kind 2 = JSON.
//!
//! DATA frames are stamped [f64be now-ms] ONCE at this ingress point and the framed Bytes shared
//! across all sessions — the same place the WS data plane stamps (gateway ingress, _common.frame),
//! so latency numbers include outbox queue time and are comparable across transports.

use std::time::Duration;

use anyhow::{bail, Result};
use bytes::{BufMut, Bytes, BytesMut};
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;
use tokio::sync::mpsc;
use tracing::{info, warn};

use crate::session::{now_ms, Hub};

const KIND_DATA: u8 = 1;
const KIND_JSON: u8 = 2;

/// Parse an LC02 packet's channel: "LC02"<u32be seq><channel>\0<payload>,
/// channel = "<topic>#<pkg>.<Type>". Port of bus.py `_parse_channel` / frame.ts `splitChannel`.
pub fn parse_channel(lc02: &[u8]) -> Option<(&str, &str)> {
    if lc02.len() < 8 || &lc02[0..4] != b"LC02" {
        return None;
    }
    let nul = lc02[8..].iter().position(|&b| b == 0)? + 8;
    let channel = std::str::from_utf8(&lc02[8..nul]).ok()?;
    Some(match channel.find('#') {
        Some(h) => (&channel[..h], &channel[h + 1..]),
        None => (channel, "?"),
    })
}

/// Run forever: connect, serve, reconnect on error.
pub async fn run(
    hub: std::sync::Arc<Hub>,
    path: String,
    mut upstream_rx: mpsc::UnboundedReceiver<Value>,
) {
    loop {
        match UnixStream::connect(&path).await {
            Ok(stream) => {
                info!(path = %path, "pipe connected");
                // The gateway (re)started: re-announce our current sub-union so on-demand
                // filtering resumes for already-connected browser sessions.
                hub.announce_subs();
                if let Err(e) = serve(hub.clone(), stream, &mut upstream_rx).await {
                    warn!("pipe connection lost: {e:#}");
                }
            }
            Err(e) => {
                warn!(path = %path, "pipe connect failed: {e}");
            }
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

async fn serve(
    hub: std::sync::Arc<Hub>,
    stream: UnixStream,
    upstream_rx: &mut mpsc::UnboundedReceiver<Value>,
) -> Result<()> {
    let (rd, mut wr) = stream.into_split();
    // Reads run in their own task: read_exact is NOT cancellation-safe, so racing it against the
    // upstream writer inside one select! would drop half-read frames and desync the pipe.
    let mut reader = tokio::spawn(read_loop(hub, rd));

    let result = loop {
        tokio::select! {
            res = &mut reader => {
                break match res {
                    Ok(r) => r.and(Err(anyhow::anyhow!("pipe read side ended"))),
                    Err(e) => Err(e.into()),
                };
            }
            msg = upstream_rx.recv() => {
                let Some(msg) = msg else { break Err(anyhow::anyhow!("upstream channel closed")) };
                let payload = serde_json::to_vec(&msg)?;
                let mut buf = BytesMut::with_capacity(5 + payload.len());
                buf.put_u32(payload.len() as u32 + 1);
                buf.put_u8(KIND_JSON);
                buf.put_slice(&payload);
                if let Err(e) = wr.write_all(&buf).await {
                    break Err(e.into());
                }
            }
        }
    };
    reader.abort();
    result
}

async fn read_loop(
    hub: std::sync::Arc<Hub>,
    mut rd: tokio::net::unix::OwnedReadHalf,
) -> Result<()> {
    loop {
        let (kind, body) = read_frame(&mut rd).await?;
        match kind {
            KIND_DATA => on_data(&hub, body),
            KIND_JSON => on_json(&hub, &body),
            other => bail!("unknown pipe frame kind {other}"),
        }
    }
}

async fn read_frame(rd: &mut (impl AsyncReadExt + Unpin)) -> Result<(u8, Bytes)> {
    let mut head = [0u8; 5];
    rd.read_exact(&mut head).await?;
    let len = u32::from_be_bytes(head[0..4].try_into().expect("4 bytes")) as usize;
    if len < 1 {
        bail!("zero-length pipe frame");
    }
    let kind = head[4];
    let mut body = vec![0u8; len - 1];
    rd.read_exact(&mut body).await?;
    Ok((kind, Bytes::from(body)))
}

fn on_data(hub: &Hub, lc02: Bytes) {
    let Some((topic, typ)) = parse_channel(&lc02) else {
        warn!("pipe DATA frame is not LC02 — dropped");
        return;
    };
    let (topic, typ) = (topic.to_string(), typ.to_string());
    // [f64be ingress-ms][LC02] — the exact frame the browser's frameToSample decodes.
    let mut framed = BytesMut::with_capacity(8 + lc02.len());
    framed.put_f64(now_ms());
    framed.put_slice(&lc02);
    hub.route(&topic, &typ, &framed.freeze());
}

fn on_json(hub: &Hub, body: &[u8]) {
    let m: Value = match serde_json::from_slice(body) {
        Ok(v) => v,
        Err(e) => {
            warn!("bad pipe JSON: {e}");
            return;
        }
    };
    match m.get("op").and_then(Value::as_str) {
        Some("hello") => {
            {
                let mut topics = hub.topics.lock().expect("hub lock");
                topics.clear();
                if let Some(list) = m.get("topics").and_then(Value::as_array) {
                    for t in list {
                        if let (Some(topic), Some(typ)) = (
                            t.get("topic").and_then(Value::as_str),
                            t.get("type").and_then(Value::as_str),
                        ) {
                            topics.insert(topic.to_string(), typ.to_string());
                        }
                    }
                }
            }
            if let Some(label) = m.get("label").and_then(Value::as_str) {
                *hub.label.lock().expect("hub lock") = label.to_string();
            }
            *hub.rpc.lock().expect("hub lock") = m.get("rpc").cloned().unwrap_or(json!([]));
            info!(
                "gateway hello: {} topics",
                hub.topics.lock().expect("hub lock").len()
            );
        }
        Some("topic") => {
            if let (Some(topic), Some(typ)) = (
                m.get("topic").and_then(Value::as_str),
                m.get("type").and_then(Value::as_str),
            ) {
                hub.topics
                    .lock()
                    .expect("hub lock")
                    .insert(topic.to_string(), typ.to_string());
                hub.broadcast_topic(topic, typ);
            }
        }
        Some("rpc-res") => {
            if let Some(sid) = m.get("sid").and_then(Value::as_u64) {
                hub.deliver_rpc_res(sid, m);
            }
        }
        // SDP offer relayed from the gateway's /rtc websocket → the WebRTC plane (rtc.rs), which
        // answers upstream with rtc-answer{rsid, sdp|error}.
        Some("rtc-offer") => {
            let (rsid, sdp) = (
                m.get("rsid").and_then(Value::as_u64),
                m.get("sdp").and_then(Value::as_str),
            );
            if let (Some(rsid), Some(sdp)) = (rsid, sdp) {
                if let Some(tx) = hub.rtc_offers.lock().expect("hub lock").as_ref() {
                    let _ = tx.send((rsid, sdp.to_string()));
                }
            }
        }
        other => warn!("unknown pipe op {other:?}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_channel_splits_topic_and_type() {
        let mut pkt = Vec::from(&b"LC02"[..]);
        pkt.extend_from_slice(&7u32.to_be_bytes());
        pkt.extend_from_slice(b"/lidar#sensor_msgs.PointCloud2\0payload-bytes");
        assert_eq!(
            parse_channel(&pkt),
            Some(("/lidar", "sensor_msgs.PointCloud2"))
        );
    }

    #[test]
    fn parse_channel_without_type_yields_question_mark() {
        let mut pkt = Vec::from(&b"LC02"[..]);
        pkt.extend_from_slice(&1u32.to_be_bytes());
        pkt.extend_from_slice(b"/bare\0x");
        assert_eq!(parse_channel(&pkt), Some(("/bare", "?")));
    }

    #[test]
    fn parse_channel_rejects_junk() {
        assert_eq!(parse_channel(b"LC03aaaa/x\0y"), None); // wrong magic
        assert_eq!(parse_channel(b"LC0"), None); // short
        let mut no_nul = Vec::from(&b"LC02"[..]);
        no_nul.extend_from_slice(&1u32.to_be_bytes());
        no_nul.extend_from_slice(b"/never-terminated");
        assert_eq!(parse_channel(&no_nul), None);
    }
}
