//! Per-browser-session state + the shared Hub. The wire protocol is pinned by the browser client
//! (packages/web/src/transports/webTransport.ts) — this must stay drop-in compatible with it.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use bytes::{BufMut, Bytes, BytesMut};
use serde_json::{json, Value};
use tokio::sync::{mpsc, Notify};

use crate::outbox::{declared_to_class, default_priority, Lane, PriorityOutbox};

/// Last-value-cache frame cap — a firehose topic must not pin tens of MB. Mirrors bus.py.
pub const LVC_MAX_BYTES: usize = 16_000_000;

pub fn now_ms() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock before epoch")
        .as_secs_f64()
        * 1000.0
}

/// Shared state between the pipe client (gateway side) and the wire sessions (browser side —
/// WebTransport and WebRTC alike).
pub struct Hub {
    pub sessions: Mutex<HashMap<u64, Arc<Session>>>,
    /// topic -> type registry, fed by the gateway's hello/topic JSON.
    pub topics: Mutex<HashMap<String, String>>,
    /// rpc manifest from the gateway hello (egress whitelist), forwarded verbatim in our hello.
    pub rpc: Mutex<Value>,
    /// Base label from the gateway hello; sessions append their wire tag (WT-rs / rtc-rs).
    pub label: Mutex<String>,
    /// JSON ops to the gateway (subs/teleop/stop/goal/rpc/disconnect). Unbounded: control-rate only.
    pub upstream: mpsc::UnboundedSender<Value>,
    /// SDP offers relayed from the gateway (rtc-offer over the pipe) → the WebRTC plane.
    pub rtc_offers: Mutex<Option<mpsc::UnboundedSender<(u64, String)>>>,
    /// Last-value cache: topic → (type, raw LC02 — pre-stamp, so a replay gets a fresh send-ms).
    /// Fed by the pipe reader (every bus frame passes it); replayed on subscribe. Mirrors bus.py.
    pub last_frames: Mutex<HashMap<String, (String, Bytes)>>,
    next_sid: AtomicU64,
}

impl Hub {
    pub fn new(upstream: mpsc::UnboundedSender<Value>) -> Self {
        Self {
            sessions: Mutex::new(HashMap::new()),
            topics: Mutex::new(HashMap::new()),
            rpc: Mutex::new(json!([])),
            label: Mutex::new("dimoscope".to_string()),
            upstream,
            rtc_offers: Mutex::new(None),
            last_frames: Mutex::new(HashMap::new()),
            next_sid: AtomicU64::new(1),
        }
    }

    pub fn add_session(
        self: &Arc<Self>,
        ctl: mpsc::UnboundedSender<Value>,
        tag: &'static str,
    ) -> Arc<Session> {
        let sid = self.next_sid.fetch_add(1, Ordering::Relaxed);
        let sess = Arc::new(Session {
            sid,
            tag,
            state: Mutex::new(SessState::default()),
            dgram: PriorityOutbox::new(),
            bulk: PriorityOutbox::new(),
            ctl,
            bulk_written: AtomicU64::new(0),
            bulk_acked: AtomicU64::new(0),
            bulk_credit: Notify::new(),
        });
        self.sessions
            .lock()
            .expect("hub lock")
            .insert(sid, sess.clone());
        sess
    }

    pub fn remove_session(&self, sid: u64) {
        self.sessions.lock().expect("hub lock").remove(&sid);
        self.send_upstream(json!({"op": "disconnect", "sid": sid}));
        self.announce_subs();
    }

    pub fn send_upstream(&self, v: Value) {
        let _ = self.upstream.send(v); // receiver lives for the process; errors only at shutdown
    }

    /// Recompute the sub-union across sessions and announce it to the gateway (on-demand filtering).
    pub fn announce_subs(&self) {
        let union: HashSet<String> = {
            let sessions = self.sessions.lock().expect("hub lock");
            sessions
                .values()
                .flat_map(|s| {
                    s.state
                        .lock()
                        .expect("session lock")
                        .subs
                        .iter()
                        .cloned()
                        .collect::<Vec<_>>()
                })
                .collect()
        };
        let mut topics: Vec<&String> = union.iter().collect();
        topics.sort();
        self.send_upstream(json!({"op": "subs", "topics": topics}));
    }

    /// Fan a bus frame out to every interested session. `framed` = [f64be ingress-ms][LC02], stamped
    /// once by the pipe reader and shared (mirrors the WS data plane's once-per-sample stamp).
    pub fn route(&self, topic: &str, typ: &str, framed: &Bytes) {
        let sessions = self.sessions.lock().expect("hub lock");
        if sessions.is_empty() {
            return;
        }
        let now = now_ms();
        let default = default_priority(topic, typ);
        for sess in sessions.values() {
            sess.offer(topic, default, framed, now);
        }
    }

    /// New-topic push (live discovery) to every session's control stream.
    pub fn broadcast_topic(&self, topic: &str, typ: &str) {
        let msg = json!({"op": "topic", "topic": topic, "type": typ});
        for sess in self.sessions.lock().expect("hub lock").values() {
            let _ = sess.ctl.send(msg.clone());
        }
    }

    pub fn topic_list(&self) -> Value {
        let topics = self.topics.lock().expect("hub lock");
        Value::Array(
            topics
                .iter()
                .map(|(t, ty)| json!({"topic": t, "type": ty}))
                .collect(),
        )
    }

    /// rpc-res from the gateway → the owning session's control stream (sid stripped).
    pub fn deliver_rpc_res(&self, sid: u64, mut m: Value) {
        if let Some(obj) = m.as_object_mut() {
            obj.remove("sid");
        }
        if let Some(sess) = self.sessions.lock().expect("hub lock").get(&sid) {
            let _ = sess.ctl.send(m);
        }
    }
}

#[derive(Default)]
struct SessState {
    subs: HashSet<String>,
    rate: HashMap<String, f64>, // per-topic maxHz cap
    last: HashMap<String, f64>, // per-topic last-delivery ms (downsample state)
    qos: HashMap<String, Lane>, // client-declared QoS overrides
}

/// QUIC datagrams are path-MTU-capped; frames over this ride the persistent bulk stream.
/// The threshold applies to the full [f64][LC02] frame.
pub const DATAGRAM_MAX: usize = 1100;

pub struct Session {
    pub sid: u64,
    /// Wire tag appended to the hello label ("WT-rs" / "rtc-rs") — bench rows self-identify.
    pub tag: &'static str,
    state: Mutex<SessState>,
    /// Datagram-size frames — drained by a task that never blocks on flow control, so a stalled
    /// 1 MB bulk write can't head-of-line-block pose.
    pub dgram: PriorityOutbox,
    /// Big frames for the persistent uni stream — its writer awaits, so QUIC flow control pushes
    /// backpressure into this outbox where conflation keeps the backlog fresh.
    pub bulk: PriorityOutbox,
    /// Control JSON to the browser (hello/topic/pong/rpc-res), written by the control-stream task.
    pub ctl: mpsc::UnboundedSender<Value>,
    /// WT bulk credit: cumulative raw bytes written to / acked consumed from the bulk uni stream
    /// (browser sends {op:"bulk-ack", n} — webTransport.ts). written − acked is the end-to-end
    /// standing queue (quinn buffer + qdisc + network + browser) the bulk drain gates on. Stays 0
    /// on the RTC plane, which has SCTP buffered_amount natively.
    pub bulk_written: AtomicU64,
    pub bulk_acked: AtomicU64,
    /// Pinged on every bulk-ack so a gated drain re-checks its budget without polling.
    pub bulk_credit: Notify,
}

impl Session {
    /// The hello the browser bootstraps on: topics + wire label + the gateway's rpc whitelist.
    /// WT clients pull it with a "list" op; the WebRTC plane pushes it when the ctl channel opens.
    pub fn hello(&self, hub: &Hub) -> Value {
        json!({
            "op": "hello",
            "topics": hub.topic_list(),
            "label": format!("{}/{}", hub.label.lock().expect("hub lock"), self.tag),
            "rpc": hub.rpc.lock().expect("hub lock").clone(),
        })
    }

    fn offer(&self, topic: &str, default: Lane, framed: &Bytes, now: f64) {
        let mut st = self.state.lock().expect("session lock");
        if !(st.subs.contains("*") || st.subs.contains(topic)) {
            return; // on-demand: unsubscribed topics never transit
        }
        let hz = st.rate.get(topic).copied().unwrap_or(0.0);
        if hz > 0.0 {
            if now - st.last.get(topic).copied().unwrap_or(0.0) < 1000.0 / hz {
                return; // per-topic downsample
            }
            st.last.insert(topic.to_string(), now);
        }
        let lane = st.qos.get(topic).copied().unwrap_or(default);
        let outbox = if framed.len() <= DATAGRAM_MAX {
            &self.dgram
        } else {
            &self.bulk
        };
        outbox.put_data(topic, lane, framed.clone());
    }

    /// Handle one control-stream op from the browser — the same control protocol as the data WS.
    pub fn on_control(&self, hub: &Hub, m: &Value) {
        let op = m.get("op").and_then(Value::as_str).unwrap_or("");
        match op {
            "subscribe" => {
                let Some(t) = m.get("topic").and_then(Value::as_str) else {
                    return;
                };
                let mut st = self.state.lock().expect("session lock");
                st.subs.insert(t.to_string());
                if let Some(hz) = m.get("maxHz").and_then(Value::as_f64) {
                    if hz > 0.0 {
                        st.rate.insert(t.to_string(), hz);
                    }
                }
                Self::apply_qos(&mut st, hub, t, m);
                drop(st);
                self.replay_last(hub, t); // last-value cache → late joiners see slow topics now
                hub.announce_subs();
            }
            "unsubscribe" => {
                let Some(t) = m.get("topic").and_then(Value::as_str) else {
                    return;
                };
                let mut st = self.state.lock().expect("session lock");
                st.subs.remove(t);
                st.rate.remove(t);
                st.last.remove(t);
                st.qos.remove(t);
                drop(st);
                hub.announce_subs();
            }
            "rate" => {
                let Some(t) = m.get("topic").and_then(Value::as_str) else {
                    return;
                };
                let hz = m.get("maxHz").and_then(Value::as_f64).unwrap_or(0.0);
                let mut st = self.state.lock().expect("session lock");
                if hz > 0.0 {
                    st.rate.insert(t.to_string(), hz);
                } else {
                    st.rate.remove(t);
                }
            }
            "list" => {
                let _ = self.ctl.send(self.hello(hub));
            }
            // WT bulk credit: cumulative bytes the browser has read off the bulk uni stream.
            // fetch_max — acks are monotonic, so a reordered/duplicate line is harmless.
            "bulk-ack" => {
                if let Some(n) = m.get("n").and_then(Value::as_u64) {
                    self.bulk_acked.fetch_max(n, Ordering::Relaxed);
                    self.bulk_credit.notify_one();
                }
            }
            "ping" => {
                // clock-sync probe: echo our clock (same host as the gateway → semantics hold)
                let _ = self.ctl.send(json!({
                    "op": "pong", "id": m.get("id").cloned().unwrap_or(Value::Null),
                    "serverTs": now_ms(),
                }));
            }
            // write path → the gateway's SafetyEgress (clamp + deadman + whitelist), keyed by our sid
            "teleop" => hub.send_upstream(json!({
                "op": "teleop", "sid": self.sid,
                "linearX": m.get("linearX").and_then(Value::as_f64).unwrap_or(0.0),
                "angularZ": m.get("angularZ").and_then(Value::as_f64).unwrap_or(0.0),
                "ttlMs": m.get("ttlMs").and_then(Value::as_f64).unwrap_or(400.0),
            })),
            "stop" => hub.send_upstream(json!({"op": "stop", "sid": self.sid})),
            "goal" => hub.send_upstream(json!({
                "op": "goal",
                "x": m.get("x").and_then(Value::as_f64).unwrap_or(0.0),
                "y": m.get("y").and_then(Value::as_f64).unwrap_or(0.0),
                "z": m.get("z").and_then(Value::as_f64).unwrap_or(0.0),
            })),
            "rpc" => hub.send_upstream(json!({
                "op": "rpc", "sid": self.sid,
                "id": m.get("id").cloned().unwrap_or(Value::Null),
                "target": m.get("target").cloned().unwrap_or(Value::Null),
                "method": m.get("method").cloned().unwrap_or(Value::Null),
                "args": m.get("args").cloned().unwrap_or(json!([])),
            })),
            _ => {}
        }
    }

    /// Replay a topic's cached frame(s) to this session — freshly stamped, routed through the
    /// normal offer() path so lane class + maxHz apply exactly like a live frame. srcTs/seq stay
    /// old inside the LC02: stale data honestly reads stale. Mirrors data.py `_replay_last`.
    fn replay_last(&self, hub: &Hub, topic: &str) {
        let cached: Vec<(String, String, Bytes)> = {
            let cache = hub.last_frames.lock().expect("hub lock");
            if topic == "*" {
                cache
                    .iter()
                    .map(|(t, (ty, b))| (t.clone(), ty.clone(), b.clone()))
                    .collect()
            } else {
                cache
                    .get(topic)
                    .map(|(ty, b)| vec![(topic.to_string(), ty.clone(), b.clone())])
                    .unwrap_or_default()
            }
        };
        let now = now_ms();
        for (t, ty, lc02) in cached {
            let mut framed = BytesMut::with_capacity(8 + lc02.len());
            framed.put_f64(now);
            framed.put_slice(&lc02);
            self.offer(&t, default_priority(&t, &ty), &framed.freeze(), now);
        }
    }

    fn apply_qos(st: &mut SessState, hub: &Hub, topic: &str, m: &Value) {
        let pr = m.get("priority").and_then(Value::as_str);
        let rel = m.get("reliability").and_then(Value::as_str);
        let depth = m.get("depth").and_then(Value::as_i64);
        if pr.is_none() && rel.is_none() && depth.is_none() {
            st.qos.remove(topic);
            return;
        }
        let typ = hub
            .topics
            .lock()
            .expect("hub lock")
            .get(topic)
            .cloned()
            .unwrap_or_default();
        let default = default_priority(topic, &typ);
        st.qos.insert(
            topic.to_string(),
            declared_to_class(pr, rel, depth, default),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hub() -> (Arc<Hub>, mpsc::UnboundedReceiver<Value>) {
        let (tx, rx) = mpsc::unbounded_channel();
        (Arc::new(Hub::new(tx)), rx)
    }

    #[test]
    fn subscribe_routes_and_unsubscribed_never_transit() {
        let (hub, _rx) = hub();
        let (ctl, _ctl_rx) = mpsc::unbounded_channel();
        let sess = hub.add_session(ctl, "WT-rs");
        let frame = Bytes::from_static(b"payload");
        hub.route("/pose", "geometry_msgs.Pose", &frame);
        sess.on_control(&hub, &json!({"op": "subscribe", "topic": "/pose"}));
        hub.route("/pose", "geometry_msgs.Pose", &frame);
        hub.route("/other", "x.Y", &frame);
        // exactly one frame queued: the post-subscribe /pose one
        assert!(futures_ready(&sess.dgram));
        assert!(!futures_ready(&sess.dgram));
    }

    fn futures_ready(ob: &PriorityOutbox) -> bool {
        // poll get() once without an executor: pick via a tiny runtime
        tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap()
            .block_on(async {
                tokio::time::timeout(std::time::Duration::from_millis(5), ob.get())
                    .await
                    .is_ok()
            })
    }

    #[test]
    fn maxhz_downsamples() {
        let (hub, _rx) = hub();
        let (ctl, _ctl_rx) = mpsc::unbounded_channel();
        let sess = hub.add_session(ctl, "WT-rs");
        // "/foo" → LANE_DEFAULT (no conflation) so each delivered frame stays queued
        sess.on_control(
            &hub,
            &json!({"op": "subscribe", "topic": "/foo", "maxHz": 10}),
        );
        let frame = Bytes::from_static(b"p");
        let now = now_ms();
        let default = default_priority("/foo", "");
        sess.offer("/foo", default, &frame, now); // delivered
        sess.offer("/foo", default, &frame, now + 10.0); // < 100ms window → dropped
        sess.offer("/foo", default, &frame, now + 110.0); // next window → delivered
        assert!(futures_ready(&sess.dgram));
        assert!(futures_ready(&sess.dgram));
        assert!(!futures_ready(&sess.dgram));
    }

    #[test]
    fn subscribe_replays_last_value_with_fresh_stamp() {
        let (hub, _rx) = hub();
        let (ctl, _ctl_rx) = mpsc::unbounded_channel();
        let sess = hub.add_session(ctl, "WT-rs");
        hub.last_frames.lock().unwrap().insert(
            "/pose".to_string(),
            ("geometry_msgs.Pose".to_string(), Bytes::from_static(b"LC02cached")),
        );
        let before = now_ms();
        sess.on_control(&hub, &json!({"op": "subscribe", "topic": "/pose"}));
        let frame = block_get(&sess.dgram).expect("subscribe must replay the cached frame");
        let stamp = f64::from_be_bytes(frame[0..8].try_into().unwrap());
        assert_eq!(&frame[8..], b"LC02cached");
        assert!(stamp >= before - 1.0, "replay must be freshly stamped");
        assert!(!futures_ready(&sess.dgram)); // exactly one replay
    }

    #[test]
    fn wildcard_subscribe_replays_all_cached_topics() {
        let (hub, _rx) = hub();
        let (ctl, _ctl_rx) = mpsc::unbounded_channel();
        let sess = hub.add_session(ctl, "WT-rs");
        {
            let mut cache = hub.last_frames.lock().unwrap();
            cache.insert("/a".to_string(), ("T".to_string(), Bytes::from_static(b"a")));
            cache.insert("/b".to_string(), ("T".to_string(), Bytes::from_static(b"b")));
        }
        sess.on_control(&hub, &json!({"op": "subscribe", "topic": "*"}));
        let mut got = vec![
            block_get(&sess.dgram).unwrap()[8..].to_vec(),
            block_get(&sess.dgram).unwrap()[8..].to_vec(),
        ];
        got.sort();
        assert_eq!(got, vec![b"a".to_vec(), b"b".to_vec()]);
        assert!(!futures_ready(&sess.dgram));
    }

    fn block_get(ob: &PriorityOutbox) -> Option<Bytes> {
        tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap()
            .block_on(async {
                tokio::time::timeout(std::time::Duration::from_millis(5), ob.get())
                    .await
                    .ok()
            })
    }

    #[test]
    fn disconnect_announces_and_sends_disconnect_op() {
        let (hub, mut rx) = hub();
        let (ctl, _ctl_rx) = mpsc::unbounded_channel();
        let sess = hub.add_session(ctl, "WT-rs");
        sess.on_control(&hub, &json!({"op": "subscribe", "topic": "/pose"}));
        let subs = rx.try_recv().unwrap();
        assert_eq!(subs["op"], "subs");
        assert_eq!(subs["topics"], json!(["/pose"]));
        hub.remove_session(sess.sid);
        let disc = rx.try_recv().unwrap();
        assert_eq!(disc["op"], "disconnect");
        assert_eq!(disc["sid"], json!(sess.sid));
        let subs2 = rx.try_recv().unwrap();
        assert_eq!(subs2["topics"], json!([]));
    }
}
