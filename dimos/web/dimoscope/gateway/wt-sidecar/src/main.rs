//! wt-sidecar — native UDP egress for the dimoscope gateway: WebTransport (QUIC :WT_PORT) and
//! WebRTC DataChannels (muxed UDP :RTC_PORT).
//!
//! Native muscle, Python brain: the Python gateway keeps the bus tap, SafetyEgress and RPC
//! whitelist, and feeds us over a unix socket (gateway/pipe.py). The wire protocols are pinned by
//! the browser clients (packages/web/src/transports/webTransport.ts + experimental/webRtcData.ts).
//!
//! Env: WT_PORT (8443) · RTC_PORT (8444) · RTC_PUBLIC_IP (host NATed → 1:1 candidate) · WT_PIPE
//! (/tmp/dimoscope-wt.sock) · WT_CERT_HASH_FILE (/tmp/dimoscope-wt-cert.hash, served by the gateway
//! /cert) · EGRESS_KBPS (optional per-session drain pacer, mirrors gateway/data.py; 0/unset = rely
//! on QUIC flow-control backpressure).

mod cert;
mod outbox;
mod pipe;
mod rtc;
mod session;

use std::sync::Arc;

use anyhow::{Context, Result};
use bytes::Bytes;
use serde_json::Value;
use tokio::sync::mpsc;
use tracing::{error, info, warn};
use wtransport::endpoint::IncomingSession;
use wtransport::error::SendDatagramError;
use wtransport::{Connection, Endpoint, ServerConfig};

use session::{Hub, Session};

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let port: u16 = env_or("WT_PORT", "8443").parse().context("WT_PORT")?;
    let pipe_path = env_or("WT_PIPE", "/tmp/dimoscope-wt.sock");
    let hash_file = env_or("WT_CERT_HASH_FILE", "/tmp/dimoscope-wt-cert.hash");
    let egress_kbps: f64 = env_or("EGRESS_KBPS", "0").parse().unwrap_or(0.0);

    // Cert FIRST: the gateway's /cert serves 503 until this file exists.
    let bundle = cert::make_identity()?;
    cert::write_hash_file(&hash_file, &bundle.hash_hex)?;
    info!(port, hash = %bundle.hash_hex, file = %hash_file, "cert ready");

    let (upstream_tx, upstream_rx) = mpsc::unbounded_channel();
    let hub = Arc::new(Hub::new(upstream_tx));
    tokio::spawn(pipe::run(hub.clone(), pipe_path, upstream_rx));

    // WebRTC plane: one muxed UDP socket, offers relayed by the gateway over the pipe.
    let rtc_port: u16 = env_or("RTC_PORT", "8444").parse().context("RTC_PORT")?;
    let rtc_public_ip = std::env::var("RTC_PUBLIC_IP").ok();
    let (offer_tx, offer_rx) = mpsc::unbounded_channel();
    *hub.rtc_offers.lock().expect("hub lock") = Some(offer_tx);
    tokio::spawn(rtc::run(hub.clone(), rtc_port, rtc_public_ip, offer_rx));

    // BBR keeps inflight ≈ BDP so a saturating bulk stream doesn't build a standing queue on a
    // shaped link — cubic bufferbloats, delaying every datagram by the queue depth. The small
    // datagram send buffer bounds staleness below the outbox for the same reason.
    let mut transport = wtransport::config::QuicTransportConfig::default();
    transport
        .congestion_controller_factory(std::sync::Arc::new(
            wtransport::quinn::congestion::BbrConfig::default(),
        ))
        .datagram_send_buffer_size(64 * 1024);

    // with_bind_default binds [::] dual-stack — required on IPv6-first macOS, where a 0.0.0.0
    // bind leaves `localhost` (resolving to ::1 first) unreachable.
    let config = ServerConfig::builder()
        .with_bind_default(port)
        .with_custom_transport(bundle.identity, transport)
        .build();
    let server = Endpoint::server(config)?;
    info!(port, "WebTransport listening (udp, dual-stack)");

    loop {
        let incoming = server.accept().await;
        let hub = hub.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_session(hub, incoming, egress_kbps).await {
                info!("session ended: {e:#}");
            }
        });
    }
}

async fn handle_session(hub: Arc<Hub>, incoming: IncomingSession, egress_kbps: f64) -> Result<()> {
    let request = incoming.await?;
    // Accept regardless of URL path — the wire protocol doesn't route on :path.
    let conn = Arc::new(request.accept().await?);
    let (ctl_tx, ctl_rx) = mpsc::unbounded_channel();
    let sess = hub.add_session(ctl_tx, "WT-rs");
    let sid = sess.sid;
    info!(sid, max_datagram = ?conn.max_datagram_size(), "session established");

    let ctl = tokio::spawn(control_stream(
        hub.clone(),
        conn.clone(),
        sess.clone(),
        ctl_rx,
    ));
    // Two drain tasks so a flow-control-stalled bulk write can never head-of-line-block datagrams;
    // the fallback channel carries the rare datagram that stopped fitting the path MTU.
    let (fallback_tx, fallback_rx) = mpsc::unbounded_channel();
    let dgrams = tokio::spawn(drain_datagrams(conn.clone(), sess.clone(), fallback_tx));
    let bulk = tokio::spawn(drain_bulk(
        conn.clone(),
        sess.clone(),
        fallback_rx,
        egress_kbps,
    ));

    // The connection closing (tab close, network drop) tears everything down; remove_session sends
    // disconnect{sid} upstream → the gateway cancels the deadman and stops the robot for this sid.
    conn.closed().await;
    ctl.abort();
    dgrams.abort();
    bulk.abort();
    hub.remove_session(sid);
    info!(sid, "session closed");
    Ok(())
}

/// The browser-opened bidi control stream: newline-delimited JSON, ops in, hello/topic/pong/rpc-res out.
async fn control_stream(
    hub: Arc<Hub>,
    conn: Arc<Connection>,
    sess: Arc<Session>,
    mut ctl_rx: mpsc::UnboundedReceiver<Value>,
) {
    let (mut send, mut recv) = match conn.accept_bi().await {
        Ok(s) => s,
        Err(e) => {
            warn!(sid = sess.sid, "no control stream: {e}");
            return;
        }
    };
    // Control outranks the bulk stream so hello/pong/rpc-res never queue behind a lidar frame.
    send.set_priority(1);

    let mut buf = Vec::with_capacity(4096);
    let mut chunk = [0u8; 4096];
    loop {
        tokio::select! {
            msg = ctl_rx.recv() => {
                let Some(msg) = msg else { return };
                let mut line = serde_json::to_vec(&msg).expect("control JSON");
                line.push(b'\n');
                if send.write_all(&line).await.is_err() {
                    return;
                }
            }
            read = recv.read(&mut chunk) => {
                let n = match read {
                    Ok(Some(n)) => n,
                    _ => return, // stream finished or connection lost
                };
                buf.extend_from_slice(&chunk[..n]);
                while let Some(nl) = buf.iter().position(|&b| b == b'\n') {
                    let line: Vec<u8> = buf.drain(..=nl).collect();
                    let line = &line[..line.len() - 1];
                    if line.iter().all(u8::is_ascii_whitespace) {
                        continue;
                    }
                    // bad JSON lines are dropped, never fatal to the session
                    if let Ok(m) = serde_json::from_slice::<Value>(line) {
                        sess.on_control(&hub, &m);
                    }
                }
                if buf.len() > 65536 {
                    buf.clear(); // runaway non-newline-terminated input → drop
                }
            }
        }
    }
}

/// Small frames on datagrams (unreliable, no HoL blocking). send_datagram never awaits flow
/// control, so pose stays fresh even while a 1 MB bulk write is stalled on a shaped link; if the
/// path MTU shrinks below a frame, it's handed to the bulk writer instead of dropped.
async fn drain_datagrams(
    conn: Arc<Connection>,
    sess: Arc<Session>,
    fallback: mpsc::UnboundedSender<Bytes>,
) {
    loop {
        let frame = sess.dgram.get().await;
        let max_dgram = conn.max_datagram_size().unwrap_or(0);
        if frame.len() <= max_dgram {
            match conn.send_datagram(&frame) {
                Ok(()) | Err(SendDatagramError::NotConnected) => {}
                Err(SendDatagramError::TooLarge) | Err(SendDatagramError::UnsupportedByPeer) => {
                    let _ = fallback.send(frame);
                }
            }
        } else {
            let _ = fallback.send(frame);
        }
    }
}

/// Big frames, length-prefixed on ONE persistent low-priority uni stream. Writes are awaited on
/// purpose: QUIC flow control pushes backpressure into the bulk outbox, where WRR + conflation keep
/// the backlog fresh. EGRESS_KBPS>0 adds an explicit pacer on top (mirrors gateway/data.py).
async fn drain_bulk(
    conn: Arc<Connection>,
    sess: Arc<Session>,
    mut fallback: mpsc::UnboundedReceiver<Bytes>,
    egress_kbps: f64,
) {
    let mut bulk: Option<wtransport::SendStream> = None;
    loop {
        // Both sources are cancellation-safe here: outbox.get() only completes when it has already
        // picked a frame, and an unbounded recv() holds no partial state.
        let frame = tokio::select! {
            f = sess.bulk.get() => f,
            Some(f) = fallback.recv() => f,
        };
        if send_bulk(&conn, &mut bulk, &frame).await.is_err() {
            error!(
                sid = sess.sid,
                "bulk stream write failed — session drain stopped"
            );
            return;
        }
        if egress_kbps > 0.0 {
            let secs = frame.len() as f64 * 8.0 / (egress_kbps * 1000.0);
            tokio::time::sleep(std::time::Duration::from_secs_f64(secs)).await;
        }
    }
}

async fn send_bulk(
    conn: &Connection,
    bulk: &mut Option<wtransport::SendStream>,
    frame: &Bytes,
) -> Result<()> {
    if bulk.is_none() {
        let s = conn.open_uni().await?.await?;
        s.set_priority(-1); // below the control stream: bulk yields to hello/pong/rpc-res
        *bulk = Some(s);
    }
    let s = bulk.as_mut().expect("bulk stream just opened");
    // [u32be length][frame] per message on the persistent stream (webTransport.ts readBulkStream)
    s.write_all(&(frame.len() as u32).to_be_bytes()).await?;
    s.write_all(frame).await?;
    Ok(())
}
