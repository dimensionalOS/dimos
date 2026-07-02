//! Port of gateway/qos.py — PriorityOutbox (priority + conflation + weighted round-robin), the
//! topic/type → lane heuristic, and the operator glob-rule layer (QOS_RULES / qos.rules.json —
//! loaded at startup by main.rs, same file the gateway reads, so both planes classify identically).
//! KEEP IN SYNC with qos.py (regex table, lane tuples, WRR weights, rule precedence); qos.py is the
//! source of truth.

use std::collections::{HashMap, VecDeque};
use std::sync::{LazyLock, Mutex};

use bytes::Bytes;
use regex::Regex;
use tokio::sync::Notify;

/// (rank, conflate, keep_last depth) — mirrors qos.py LANE_* tuples.
pub type Lane = (usize, bool, usize);

pub const LANE_COMMAND: Lane = (3, false, 8);
pub const LANE_SENSOR: Lane = (2, true, 1);
pub const LANE_DEFAULT: Lane = (1, false, 16);
pub const LANE_BULK: Lane = (0, true, 1);
pub const CONTROL: Lane = (3, false, 256);

const RANK: usize = 4;
const WEIGHTS: [u32; RANK] = [1, 2, 4, 8]; // indexed by class: 0→1, 1→2, 2→4, 3→8

macro_rules! re {
    ($pat:expr) => {
        LazyLock::new(|| Regex::new($pat).expect("static regex"))
    };
}

// qos.py regex table (case-insensitive, topic-segment anchored). BULK is checked before SENSOR.
static CMD: LazyLock<Regex> =
    re!(r"(?i)(^|/)(cmd_vel|cmd|teleop|goal|clicked_point|move_base|nav_goal)(/|$)");
static BULK: LazyLock<Regex> = re!(
    r"(?i)(^|/)(lidar|laser|scan|points?|point_?cloud|cloud|camera|image|img|depth|rgb|map|costmap|occupancy|grid)(/|$)"
);
static SENSOR: LazyLock<Regex> =
    re!(r"(?i)(^|/)(pose|odom|imu|tf|joint|battery|twist|velocity|state|wrench|p\d+)(/|$)");
static CMD_T: LazyLock<Regex> = re!(r"(?i)(Twist|Goal)");
static BULK_T: LazyLock<Regex> =
    re!(r"(?i)(PointCloud|LaserScan|Image|OccupancyGrid|CompressedImage)");
static SENSOR_T: LazyLock<Regex> =
    re!(r"(?i)(Pose|Odometry|Imu|Transform|JointState|Quaternion|Vector3)");

// Operator rules (qos.rules.json): (pattern-has-#, compiled glob, lane), first match wins —
// checked before the heuristic, mirroring qos.py _rule_lane. Set once at startup.
static RULES: LazyLock<Mutex<Vec<(bool, Regex, Lane)>>> = LazyLock::new(|| Mutex::new(Vec::new()));

fn lane_by_name(name: &str) -> Option<Lane> {
    match name {
        "command" => Some(LANE_COMMAND),
        "sensor" => Some(LANE_SENSOR),
        "default" => Some(LANE_DEFAULT),
        "bulk" => Some(LANE_BULK),
        _ => None,
    }
}

/// fnmatch → anchored regex: `*` → `.*`, `?` → `.`, `[seq]`/`[!seq]` classes pass through
/// (`!` → `^`), everything else escaped. Covers the fnmatch subset qos.py accepts.
fn fnmatch_to_regex(pat: &str) -> Option<Regex> {
    let mut out = String::from("(?s)^");
    let mut chars = pat.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '*' => out.push_str(".*"),
            '?' => out.push('.'),
            '[' => {
                out.push('[');
                if chars.peek() == Some(&'!') {
                    chars.next();
                    out.push('^');
                }
                for c2 in chars.by_ref() {
                    out.push(c2);
                    if c2 == ']' {
                        break;
                    }
                }
            }
            c => out.push_str(&regex::escape(&c.to_string())),
        }
    }
    out.push('$');
    Regex::new(&out).ok()
}

/// Install the operator rules: (fnmatch pattern, lane name) pairs, order = precedence.
/// Unknown lanes / uncompilable patterns are skipped. Returns how many rules took effect.
pub fn set_qos_rules(pairs: &[(String, String)]) -> usize {
    let compiled: Vec<(bool, Regex, Lane)> = pairs
        .iter()
        .filter_map(|(pat, lane)| {
            Some((pat.contains('#'), fnmatch_to_regex(pat)?, lane_by_name(lane)?))
        })
        .collect();
    let n = compiled.len();
    *RULES.lock().expect("rules lock") = compiled;
    n
}

/// First matching operator rule → its lane, else None. A '#' in the pattern matches
/// "<topic>#<type>" (classify by message type) — same convention as qos.py.
fn rule_lane(topic: &str, typ: &str) -> Option<Lane> {
    let rules = RULES.lock().expect("rules lock");
    for (with_type, re, lane) in rules.iter() {
        let matched = if *with_type {
            re.is_match(&format!("{topic}#{typ}"))
        } else {
            re.is_match(topic)
        };
        if matched {
            return Some(*lane);
        }
    }
    None
}

/// qos.py default_priority: operator rules first, then the name/type heuristic.
pub fn default_priority(topic: &str, typ: &str) -> Lane {
    if let Some(lane) = rule_lane(topic, typ) {
        return lane;
    }
    if CMD.is_match(topic) || CMD_T.is_match(typ) {
        LANE_COMMAND
    } else if BULK.is_match(topic) || BULK_T.is_match(typ) {
        LANE_BULK // heavy/big beats the generic sensor match
    } else if SENSOR.is_match(topic) || SENSOR_T.is_match(typ) {
        LANE_SENSOR
    } else {
        LANE_DEFAULT
    }
}

/// qos.py declared_to_class: merge the client's declared QoS (subscribe op fields) onto the default —
/// only fields the client set are overridden; "best-effort" → conflate (latest-wins).
pub fn declared_to_class(
    priority: Option<&str>,
    reliability: Option<&str>,
    depth: Option<i64>,
    default: Lane,
) -> Lane {
    let rank = match priority {
        Some("low") => 0,
        Some("normal") => 1,
        Some("high") => 2,
        Some("critical") => 3,
        Some(_) => default.0, // unknown string → default, like dict.get
        None => default.0,
    };
    let conflate = match reliability {
        Some(r) => r == "best-effort",
        None => default.1,
    };
    // qos.py: `int(depth) if depth else default` — 0 is falsy and falls back too.
    let d = match depth {
        Some(d) if d > 0 => d as usize,
        _ => default.2,
    };
    (rank, conflate, d)
}

struct Bucket {
    frames: VecDeque<Bytes>,
    maxlen: usize, // fixed at first put, like Python's deque(maxlen=...)
}

#[derive(Default)]
struct Class {
    order: VecDeque<String>, // round-robin order across topics (front = next served)
    buckets: HashMap<String, Bucket>,
}

struct Inner {
    cls: [Class; RANK],
    credits: [u32; RANK],
}

/// Per-session outbox: put never blocks (conflation/keep_last shed instead); get awaits frames and
/// drains by weighted round-robin with a non-starvation floor for the lowest class.
pub struct PriorityOutbox {
    inner: Mutex<Inner>,
    notify: Notify,
}

impl Default for PriorityOutbox {
    fn default() -> Self {
        Self::new()
    }
}

impl PriorityOutbox {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(Inner {
                cls: Default::default(),
                credits: WEIGHTS,
            }),
            notify: Notify::new(),
        }
    }

    /// JSON control (hello/topic/rpc-res) — top priority, generously buffered. Unused on the WT path
    /// (control rides its own QUIC stream, written directly) but kept for parity with qos.py.
    #[allow(dead_code)]
    pub fn put_control(&self, item: Bytes) {
        self.put("\x00ctl", CONTROL, item);
    }

    pub fn put_data(&self, topic: &str, lane: Lane, item: Bytes) {
        self.put(topic, lane, item);
    }

    fn put(&self, topic: &str, (prio, conflate, depth): Lane, item: Bytes) {
        {
            let mut inner = self.inner.lock().expect("outbox lock");
            let class = &mut inner.cls[prio];
            let bucket = match class.buckets.get_mut(topic) {
                Some(b) => b,
                None => {
                    class.order.push_back(topic.to_string());
                    class.buckets.entry(topic.to_string()).or_insert(Bucket {
                        frames: VecDeque::new(),
                        maxlen: if conflate { 1 } else { depth.max(1) },
                    })
                }
            };
            if bucket.frames.len() >= bucket.maxlen {
                bucket.frames.pop_front(); // conflate → overwrite; reliable → bounded keep_last
            }
            bucket.frames.push_back(item);
        }
        self.notify.notify_one();
    }

    pub async fn get(&self) -> Bytes {
        loop {
            if let Some(item) = self.pick() {
                return item;
            }
            self.notify.notified().await;
        }
    }

    fn pick(&self) -> Option<Bytes> {
        let mut inner = self.inner.lock().expect("outbox lock");
        // weighted round-robin: serve a class while it has credit; refill when all credited drained
        for _ in 0..2 {
            // at most one refill pass
            for c in (0..RANK).rev() {
                if !inner.cls[c].order.is_empty() && inner.credits[c] > 0 {
                    inner.credits[c] -= 1;
                    return Some(Self::pop_rr(&mut inner.cls[c]));
                }
            }
            if inner.cls.iter().any(|c| !c.order.is_empty()) {
                inner.credits = WEIGHTS; // refill and serve highest non-empty (the floor)
                continue;
            }
            return None;
        }
        None
    }

    fn pop_rr(class: &mut Class) -> Bytes {
        // round-robin across topics within a class: front topic serves one frame, rotates to the back
        let topic = class.order.pop_front().expect("non-empty class");
        let bucket = class
            .buckets
            .get_mut(&topic)
            .expect("bucket for ordered topic");
        let item = bucket.frames.pop_front().expect("non-empty bucket");
        if bucket.frames.is_empty() {
            class.buckets.remove(&topic); // drained topics are deleted, not left empty
        } else {
            class.order.push_back(topic);
        }
        item
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn b(s: &str) -> Bytes {
        Bytes::copy_from_slice(s.as_bytes())
    }

    fn fill(ob: &PriorityOutbox, topic: &str, prio: usize, n: usize) {
        for i in 0..n {
            ob.put_data(topic, (prio, false, 64), b(&format!("{topic}:{i}")));
        }
    }

    fn drain(ob: &PriorityOutbox, n: usize) -> Vec<Bytes> {
        let mut out = Vec::new();
        for _ in 0..n {
            match ob.pick() {
                Some(item) => out.push(item),
                None => break,
            }
        }
        out
    }

    // One serial test for the whole rules layer: set_qos_rules REPLACES the global table, so
    // separate #[test]s would race under cargo's parallel threads. Patterns are namespaced
    // (/rulez-…) so the concurrently-running heuristic tests can never collide with them.
    #[test]
    fn operator_rules_layer() {
        // rule beats heuristic; empty table restores it
        assert_eq!(set_qos_rules(&[("/rulez-debug/*".to_string(), "bulk".to_string())]), 1);
        assert_eq!(default_priority("/rulez-debug/anything", ""), LANE_BULK);
        set_qos_rules(&[]);
        assert_eq!(default_priority("/rulez-debug/anything", ""), LANE_DEFAULT);

        // '#' pattern classifies by "<topic>#<type>"
        set_qos_rules(&[("*#rulez_msgs.DriveCmd".to_string(), "command".to_string())]);
        assert_eq!(default_priority("/whatever-rulez", "rulez_msgs.DriveCmd"), LANE_COMMAND);
        assert_eq!(default_priority("/whatever-rulez", "rulez_msgs.Other"), LANE_DEFAULT);

        // first match wins; unknown lane names are skipped
        let n = set_qos_rules(&[
            ("/rulez-x/*".to_string(), "sensor".to_string()),
            ("/rulez-x/*".to_string(), "bulk".to_string()),
            ("/rulez-y".to_string(), "no-such-lane".to_string()),
        ]);
        assert_eq!(n, 2);
        assert_eq!(default_priority("/rulez-x/a", ""), LANE_SENSOR);

        // fnmatch translation: ? = exactly one char, [ab] classes pass through
        set_qos_rules(&[
            ("/rulez-q?".to_string(), "bulk".to_string()),
            ("/rulez-[ab]".to_string(), "sensor".to_string()),
        ]);
        assert_eq!(default_priority("/rulez-q1", ""), LANE_BULK);
        assert_eq!(default_priority("/rulez-q12", ""), LANE_DEFAULT);
        assert_eq!(default_priority("/rulez-a", ""), LANE_SENSOR);
        assert_eq!(default_priority("/rulez-c", ""), LANE_DEFAULT);
        set_qos_rules(&[]);
    }

    #[test]
    fn higher_class_served_first() {
        let ob = PriorityOutbox::new();
        fill(&ob, "/lo", 0, 1);
        fill(&ob, "/hi", 3, 1);
        assert_eq!(drain(&ob, 2), vec![b("/hi:0"), b("/lo:0")]);
    }

    #[test]
    fn wrr_ratio_and_low_class_floor() {
        // mirror test_outbox.py: per credit cycle 8×class-3 then 1×class-0, repeating
        let ob = PriorityOutbox::new();
        fill(&ob, "/hi", 3, 20);
        fill(&ob, "/lo", 0, 20);
        let picked = drain(&ob, 18);
        let mut expect: Vec<Bytes> = (0..8).map(|i| b(&format!("/hi:{i}"))).collect();
        expect.push(b("/lo:0"));
        expect.extend((8..16).map(|i| b(&format!("/hi:{i}"))));
        expect.push(b("/lo:1"));
        assert_eq!(picked, expect);
    }

    #[test]
    fn conflation_keeps_only_latest() {
        let ob = PriorityOutbox::new();
        for payload in ["a", "b", "c"] {
            ob.put_data("/img", (0, true, 1), b(payload));
        }
        assert_eq!(drain(&ob, 5), vec![b("c")]);
    }

    #[test]
    fn reliable_keep_last_bounds_the_deque() {
        let ob = PriorityOutbox::new();
        for i in 0..5 {
            ob.put_data("/r", (1, false, 3), b(&i.to_string()));
        }
        assert_eq!(drain(&ob, 5), vec![b("2"), b("3"), b("4")]);
    }

    #[test]
    fn round_robin_across_topics_within_a_class() {
        let ob = PriorityOutbox::new();
        for i in 0..2 {
            ob.put_data("/a", (2, false, 8), b(&format!("a{i}")));
            ob.put_data("/b", (2, false, 8), b(&format!("b{i}")));
        }
        assert_eq!(drain(&ob, 4), vec![b("a0"), b("b0"), b("a1"), b("b1")]);
    }

    #[test]
    fn control_beats_data() {
        let ob = PriorityOutbox::new();
        fill(&ob, "/bulk", 0, 3);
        ob.put_control(b(r#"{"op":"topic"}"#));
        assert_eq!(drain(&ob, 1), vec![b(r#"{"op":"topic"}"#)]);
    }

    #[test]
    fn empty_pick_returns_none_and_memory_is_freed() {
        let ob = PriorityOutbox::new();
        fill(&ob, "/a", 1, 2);
        drain(&ob, 2);
        assert!(ob.pick().is_none());
        let inner = ob.inner.lock().unwrap();
        assert!(inner
            .cls
            .iter()
            .all(|c| c.buckets.is_empty() && c.order.is_empty()));
    }

    #[tokio::test]
    async fn get_blocks_until_put_wakes_it() {
        use std::sync::Arc;
        let ob = Arc::new(PriorityOutbox::new());
        let ob2 = ob.clone();
        let task = tokio::spawn(async move { ob2.get().await });
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        assert!(!task.is_finished());
        ob.put_data("/a", (1, false, 4), b("x"));
        let got = tokio::time::timeout(std::time::Duration::from_secs(1), task)
            .await
            .unwrap()
            .unwrap();
        assert_eq!(got, b("x"));
    }

    #[test]
    fn default_priority_matches_qos_py() {
        assert_eq!(default_priority("/cmd_vel", ""), LANE_COMMAND);
        assert_eq!(default_priority("/go2/camera/rgb", ""), LANE_BULK);
        assert_eq!(default_priority("/pose", ""), LANE_SENSOR);
        assert_eq!(default_priority("/camera/pose", ""), LANE_BULK); // BULK beats SENSOR
        assert_eq!(default_priority("/whatever", ""), LANE_DEFAULT);
        assert_eq!(default_priority("/x", "geometry_msgs.Twist"), LANE_COMMAND);
        assert_eq!(default_priority("/x", "sensor_msgs.PointCloud2"), LANE_BULK);
        assert_eq!(default_priority("/x", "geometry_msgs.Pose"), LANE_SENSOR);
        assert_eq!(default_priority("/p0", ""), LANE_SENSOR); // bench topics p0..pN
    }

    #[test]
    fn declared_to_class_merges_onto_default() {
        let d = LANE_DEFAULT;
        assert_eq!(
            declared_to_class(Some("critical"), None, None, d),
            (3, false, 16)
        );
        assert_eq!(
            declared_to_class(None, Some("best-effort"), None, d),
            (1, true, 16)
        );
        assert_eq!(
            declared_to_class(None, Some("reliable"), Some(4), d),
            (1, false, 4)
        );
        assert_eq!(declared_to_class(None, None, Some(0), d), d); // 0 is falsy in qos.py
        assert_eq!(declared_to_class(Some("bogus"), None, None, d), d);
    }
}
