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
//! on QUIC flow-control backpressure) · QOS_RULES (operator lane rules, default ./qos.rules.json —
//! the same file the gateway loads, so both planes classify identically) · WT_BULK_TARGET_MS (250;
//! bulk credit gate: keep ≤ this many ms of standing bulk queue, 0 = off) · WT_BULK_MIN (65536;
//! credit budget floor in bytes) · WT_STATS_S (0; >0 = per-session queue diagnostics every N s).

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

use session::{now_ms, Hub, Session};

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Operator lane rules (qos.rules.json) — same file, same precedence as the gateway (qos.py), so a
/// topic classifies identically on every wire. Absent file = pure heuristic (the common case).
/// The serve tasks run from the dimoscope root, where the default relative path matches the
/// gateway's `<dimoscope>/qos.rules.json`; QOS_RULES overrides for other layouts.
fn load_qos_rules(path: &str) {
    let Ok(raw) = std::fs::read_to_string(path) else {
        info!(path, "no qos rules (heuristic lanes only)");
        return;
    };
    let entries: Vec<Value> = match serde_json::from_str(&raw) {
        Ok(v) => v,
        Err(e) => {
            warn!(path, %e, "qos rules unreadable — heuristic lanes only");
            return;
        }
    };
    let pairs: Vec<(String, String)> = entries
        .iter()
        .filter_map(|r| {
            Some((
                r.get("topic")?.as_str()?.to_string(),
                r.get("lane")?.as_str()?.to_string(),
            ))
        })
        .collect();
    let n = outbox::set_qos_rules(&pairs);
    info!(path, count = n, "loaded qos rules");
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
    // send_window bounds the connection's send buffer. quinn's ~10 MB default lets write_all accept
    // frame after frame instantly, so a saturating bulk topic piles up in quinn's buffer *past* the
    // outbox — measured 5+ s of stale backlog on a shaped link. Capping it to ~1.5 MB paces the
    // drain to the link rate: write_all blocks, the backlog stays in the outbox where conflation
    // keeps only the freshest frame. 1.5 MB > clean-WAN BDP (~0.6 MB at 20 MB/s × 30 ms) so clean
    // throughput is unaffected.
    let send_window: u64 = env_or("WT_SEND_WINDOW", "1500000").parse().unwrap_or(1_500_000);
    // Datagram freshness: drop a pose frame older than this at drain time (post-stall staleness
    // bound), and keep quinn's datagram send buffer small so newest-wins by drop-oldest.
    let dgram_ttl_ms: f64 = env_or("WT_DGRAM_TTL_MS", "200").parse().unwrap_or(200.0);
    let dgram_buf: usize = env_or("WT_DGRAM_BUF", "16384").parse().unwrap_or(16384);
    // Belt-and-braces: explicitly advertise datagram support (an unset receive buffer would leave
    // max_datagram_size() None and every small frame undeliverable). Measured sessions negotiate
    // Some(~1295) with quinn's defaults too — this pins the behavior rather than changing it.
    let dgram_recv_buf: usize = env_or("WT_DGRAM_RECV_BUF", "1048576").parse().unwrap_or(1_048_576);
    // Bulk credit gate: the browser acks consumed bulk-stream bytes (bulk-ack op) and the drain
    // keeps written − acked ≤ ack-rate × target — i.e. at most ~target ms of standing queue across
    // quinn buffer + qdisc + network, whatever the link rate. A static send_window can't do this:
    // 1.5 MB is fine on a clean WAN but 6 s of queue at 2 Mbit. 0 disables the gate (A/B baseline).
    let knobs = SessionKnobs {
        egress_kbps,
        dgram_ttl_ms,
        bulk_target_ms: env_or("WT_BULK_TARGET_MS", "250").parse().unwrap_or(250.0),
        bulk_min: env_or("WT_BULK_MIN", "65536").parse().unwrap_or(65536),
        stats_s: env_or("WT_STATS_S", "0").parse().unwrap_or(0.0),
    };
    load_qos_rules(&env_or("QOS_RULES", "qos.rules.json"));

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
    // shaped link — cubic bufferbloats, delaying every datagram by the queue depth. send_window
    // caps the send buffer so bulk backlog stays in the outbox (conflated fresh); the small
    // datagram send buffer bounds pose staleness the same way (drop-oldest = newest-wins).
    let mut transport = wtransport::config::QuicTransportConfig::default();
    transport
        .congestion_controller_factory(std::sync::Arc::new(
            wtransport::quinn::congestion::BbrConfig::default(),
        ))
        .send_window(send_window)
        .datagram_send_buffer_size(dgram_buf)
        .datagram_receive_buffer_size(Some(dgram_recv_buf));

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
            if let Err(e) = handle_session(hub, incoming, knobs).await {
                info!("session ended: {e:#}");
            }
        });
    }
}

/// Per-session drain tuning, read once from env in main().
#[derive(Clone, Copy)]
struct SessionKnobs {
    egress_kbps: f64,
    dgram_ttl_ms: f64,
    bulk_target_ms: f64,
    bulk_min: u64,
    stats_s: f64,
}

async fn handle_session(hub: Arc<Hub>, incoming: IncomingSession, knobs: SessionKnobs) -> Result<()> {
    let request = incoming.await?;
    // Accept regardless of URL path — the wire protocol doesn't route on :path.
    let conn = Arc::new(request.accept().await?);
    let (ctl_tx, ctl_rx) = mpsc::unbounded_channel();
    let sess = hub.add_session(ctl_tx, "WT-rs");
    let sid = sess.sid;
    // max_datagram_size() is logged live by session_stats (WT_STATS_S), not sampled here.
    info!(sid, "session established");

    let ctl = tokio::spawn(control_stream(
        hub.clone(),
        conn.clone(),
        sess.clone(),
        ctl_rx,
    ));
    // Two independent drain tasks: small lanes on datagrams (drain_datagrams, drop-if-undeliverable),
    // big frames on the credit-gated bulk stream (drain_bulk) — a stalled bulk write can never
    // head-of-line-block a datagram.
    let dgrams = tokio::spawn(drain_datagrams(conn.clone(), sess.clone(), knobs.dgram_ttl_ms));
    let bulk = tokio::spawn(drain_bulk(conn.clone(), sess.clone(), knobs));
    let stats = (knobs.stats_s > 0.0)
        .then(|| tokio::spawn(session_stats(conn.clone(), sess.clone(), knobs.stats_s)));

    // The connection closing (tab close, network drop) tears everything down; remove_session sends
    // disconnect{sid} upstream → the gateway cancels the deadman and stops the robot for this sid.
    conn.closed().await;
    ctl.abort();
    dgrams.abort();
    bulk.abort();
    if let Some(s) = stats {
        s.abort();
    }
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

/// Small frames on datagrams (unreliable, no HoL blocking). send_datagram never awaits flow control,
/// so pose stays fresh even while a 1 MB bulk write is stalled on a shaped link. A frame that can't
/// be a datagram (MTU shrank, or datagrams not negotiated → max_datagram_size None) is DROPPED, not
/// queued onto the reliable bulk stream: the dgram outbox is conflate/sensor (pose), where freshness
/// beats completeness, and a fallback would couple the small lane to the bulk queue's latency.
/// (The ×40 pose bufferbloat itself was bulk bytes in flight standing in the qdisc — datagrams were
/// negotiated and sent all along, transmitting behind that queue; the credit gate is the cure.)
async fn drain_datagrams(conn: Arc<Connection>, sess: Arc<Session>, ttl_ms: f64) {
    let mut dropped = 0u64;
    loop {
        let frame = sess.dgram.get().await;
        // frame = [f64be ingress-ms][LC02]; skip if it aged past the TTL waiting to drain.
        if ttl_ms > 0.0 && frame.len() >= 8 {
            let stamp = f64::from_be_bytes(frame[..8].try_into().expect("8 bytes"));
            if now_ms() - stamp > ttl_ms {
                continue;
            }
        }
        let max_dgram = conn.max_datagram_size().unwrap_or(0);
        let send = matches!(dgram_action(frame.len(), max_dgram), DgramAction::Send)
            && !matches!(
                conn.send_datagram(&frame),
                Err(SendDatagramError::TooLarge) | Err(SendDatagramError::UnsupportedByPeer),
            );
        if !send {
            drop_datagram(&sess, &mut dropped, max_dgram, frame.len());
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
enum DgramAction {
    Send,
    Drop,
}

/// Whether a datagram-outbox frame can leave as a QUIC datagram. `max_dgram == 0` = datagrams not
/// negotiated yet (`max_datagram_size()` is None) → Drop rather than couple the small lane to the
/// bulk queue. Pure so it's unit-tested without a live Connection.
fn dgram_action(frame_len: usize, max_dgram: usize) -> DgramAction {
    if max_dgram > 0 && frame_len <= max_dgram {
        DgramAction::Send
    } else {
        DgramAction::Drop
    }
}

/// Power-of-two rate-limited warn so an unnegotiated session (drop-every-frame) can't flood the log.
fn drop_datagram(sess: &Session, dropped: &mut u64, max_dgram: usize, frame_len: usize) {
    *dropped += 1;
    if dropped.is_power_of_two() {
        warn!(
            sid = sess.sid,
            dropped, max_dgram, frame_len, "datagram undeliverable — dropped (freshness > completeness)"
        );
    }
}

/// Per-chunk granularity of the credit gate inside one frame (mirrors rtc.rs CHUNK): the gate can
/// close mid-frame, but a pulled frame is always finished — the [u32be len] framing forbids
/// abandoning it — so the worst-case overshoot past the budget is one chunk.
const BULK_CHUNK: usize = 65536;

/// Big frames, length-prefixed on ONE persistent low-priority uni stream. Writes are awaited on
/// purpose: QUIC flow control pushes backpressure into the bulk outbox, where WRR + conflation keep
/// the backlog fresh. The credit gate (wait_credit) bounds the standing queue *below* quinn on
/// rate-capped links, where send_window alone is seconds of bufferbloat. EGRESS_KBPS>0 adds an
/// explicit pacer on top (mirrors gateway/data.py).
async fn drain_bulk(conn: Arc<Connection>, sess: Arc<Session>, knobs: SessionKnobs) {
    let mut bulk: Option<wtransport::SendStream> = None;
    let mut rate = AckRate::default();
    loop {
        // Gate BEFORE pulling: while credit is exhausted the backlog stays in the outbox, where
        // per-topic conflation keeps only the freshest frame.
        wait_credit(&sess, &mut rate, &knobs).await;
        // Cancellation-safe: outbox.get() only completes once it has already picked a frame.
        let frame = sess.bulk.get().await;
        if send_bulk(&conn, &mut bulk, &frame, &sess, &mut rate, &knobs)
            .await
            .is_err()
        {
            error!(
                sid = sess.sid,
                "bulk stream write failed — session drain stopped"
            );
            return;
        }
        if knobs.egress_kbps > 0.0 {
            let secs = frame.len() as f64 * 8.0 / (knobs.egress_kbps * 1000.0);
            tokio::time::sleep(std::time::Duration::from_secs_f64(secs)).await;
        }
    }
}

async fn send_bulk(
    conn: &Connection,
    bulk: &mut Option<wtransport::SendStream>,
    frame: &Bytes,
    sess: &Session,
    rate: &mut AckRate,
    knobs: &SessionKnobs,
) -> Result<()> {
    if bulk.is_none() {
        let s = conn.open_uni().await?.await?;
        s.set_priority(-1); // below the control stream: bulk yields to hello/pong/rpc-res
        *bulk = Some(s);
    }
    let s = bulk.as_mut().expect("bulk stream just opened");
    // [u32be length][frame] per message on the persistent stream (webTransport.ts readBulkStream)
    s.write_all(&(frame.len() as u32).to_be_bytes()).await?;
    sess.bulk_written
        .fetch_add(4, std::sync::atomic::Ordering::Relaxed);
    let mut off = 0;
    while off < frame.len() {
        if off > 0 {
            wait_credit(sess, rate, knobs).await; // drain_bulk gated before the first chunk
        }
        let end = (off + BULK_CHUNK).min(frame.len());
        s.write_all(&frame[off..end]).await?;
        sess.bulk_written
            .fetch_add((end - off) as u64, std::sync::atomic::Ordering::Relaxed);
        off = end;
    }
    Ok(())
}

/// EMA of the browser's bulk-ack byte rate (bytes/ms), sampled from the cumulative bulk_acked
/// counter at irregular intervals. τ ≈ 500 ms: fast enough to open the budget during ramp-up,
/// slow enough to ride out the 32 KB ack quantization.
#[derive(Default)]
struct AckRate {
    last_acked: u64,
    last_ms: f64,
    per_ms: f64,
    /// Set when a client never acked within the pre-ack grace — a legacy build without bulk-ack.
    /// Only consulted while acked == 0; the first real ack switches to the normal budget path.
    legacy: bool,
    wait_start_ms: f64,
}

impl AckRate {
    const TAU_MS: f64 = 500.0;

    fn update(&mut self, acked: u64, now: f64) {
        if self.last_ms == 0.0 {
            (self.last_acked, self.last_ms) = (acked, now);
            return;
        }
        let dt = now - self.last_ms;
        if dt < 20.0 {
            return; // too short a window to divide meaningfully
        }
        let inst = acked.saturating_sub(self.last_acked) as f64 / dt;
        let alpha = 1.0 - (-dt / Self::TAU_MS).exp();
        self.per_ms += alpha * (inst - self.per_ms);
        (self.last_acked, self.last_ms) = (acked, now);
    }
}

/// Bulk credit gate: block while outstanding = written − acked exceeds the budget
/// max(WT_BULK_MIN, ack-rate × WT_BULK_TARGET_MS) — i.e. keep at most ~target ms of standing bulk
/// queue across quinn's buffer, the qdisc and the network, whatever the link rate. The receiver's
/// consumption rate is the only trustworthy congestion signal here (quinn-BBR's cwnd was measured
/// ≥ send_window on a 2 Mbit shaped link, its min-rtt filter poisoned by its own standing queue).
///
/// Before the first ack the budget is the WT_BULK_MIN floor — assuming the worst-case link until
/// the receiver proves otherwise. Opening the gate pre-ack instead let the drain stuff the whole
/// send_window in the ~400 ms before the first ack, and that one-time queue took ~6 s to drain at
/// 2 Mbit, poisoning every lane's p95 for the entire run. A client that never acks (legacy build)
/// gets one 3 s gated grace, then today's send_window-bounded behavior. Requires the client ack
/// quantum (32 KB) < WT_BULK_MIN or the pre-ack gate would deadlock before the first ack fires.
async fn wait_credit(sess: &Session, rate: &mut AckRate, knobs: &SessionKnobs) {
    if knobs.bulk_target_ms <= 0.0 {
        return;
    }
    loop {
        let written = sess
            .bulk_written
            .load(std::sync::atomic::Ordering::Relaxed);
        let acked = sess.bulk_acked.load(std::sync::atomic::Ordering::Relaxed);
        let budget = if acked == 0 {
            if rate.legacy {
                return;
            }
            knobs.bulk_min
        } else {
            rate.update(acked, now_ms());
            (rate.per_ms * knobs.bulk_target_ms).max(knobs.bulk_min as f64) as u64
        };
        if written.saturating_sub(acked) < budget {
            return;
        }
        if acked == 0 {
            let now = now_ms();
            if rate.wait_start_ms == 0.0 {
                rate.wait_start_ms = now;
            } else if now - rate.wait_start_ms > 3000.0 {
                rate.legacy = true;
                return;
            }
        }
        // The 100 ms timeout kills any missed-notify race and refreshes the budget while acks
        // are sparse (a stalled reader parks us here without blocking datagrams or control).
        let _ = tokio::time::timeout(
            std::time::Duration::from_millis(100),
            sess.bulk_credit.notified(),
        )
        .await;
    }
}

/// WT_STATS_S > 0: one line every interval answering "where does the standing queue live?" —
/// written−acked spans quinn's send buffer + qdisc + network + browser; cwnd/rtt say what quinn's
/// congestion controller believes about the link; udp_tx_Bps is what actually left the socket.
async fn session_stats(conn: Arc<Connection>, sess: Arc<Session>, interval_s: f64) {
    let q = conn.quic_connection();
    let mut last_tx = 0u64;
    let mut tick = tokio::time::interval(std::time::Duration::from_secs_f64(interval_s));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tick.tick().await;
        let stats = q.stats();
        let tx = stats.udp_tx.bytes;
        let written = sess
            .bulk_written
            .load(std::sync::atomic::Ordering::Relaxed);
        let acked = sess.bulk_acked.load(std::sync::atomic::Ordering::Relaxed);
        info!(
            sid = sess.sid,
            cwnd = stats.path.cwnd,
            rtt_ms = stats.path.rtt.as_millis() as u64,
            udp_tx_bps = ((tx.saturating_sub(last_tx)) as f64 / interval_s) as u64,
            dgram_buf_space = q.datagram_send_buffer_space(),
            max_dgram = ?q.max_datagram_size(),
            bulk_written = written,
            bulk_acked = acked,
            outstanding = written.saturating_sub(acked),
            "wt-stats"
        );
        last_tx = tx;
    }
}

#[cfg(test)]
mod tests {
    use super::{dgram_action, DgramAction};

    #[test]
    fn dgram_action_sends_only_when_negotiated_and_fits() {
        // datagrams not negotiated (max_datagram_size None → 0): drop, never fall back to bulk
        assert_eq!(dgram_action(143, 0), DgramAction::Drop);
        // negotiated and fits: send
        assert_eq!(dgram_action(143, 1200), DgramAction::Send);
        // negotiated but frame larger than the path can carry: drop
        assert_eq!(dgram_action(1300, 1200), DgramAction::Drop);
        // exact boundary fits
        assert_eq!(dgram_action(1200, 1200), DgramAction::Send);
    }
}
