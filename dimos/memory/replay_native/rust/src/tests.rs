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

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::mpsc::channel;

use rusqlite::Connection;
use tempfile::TempDir;

use super::*;

const T0: f64 = 1_000.0;

/// `(ts, stored bytes)` rows of one stream.
type Rows = Vec<(f64, Vec<u8>)>;

/// A memory `.db` with the tables the Python SqliteStore writes: `_streams`, `<name>`, `<name>_blob`.
fn write_db(path: &Path, streams: &[(&str, &str, Rows)]) {
    let db = Connection::open(path).unwrap();
    db.execute_batch("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT)")
        .unwrap();
    for (name, codec, rows) in streams {
        let config = format!(r#"{{"payload_module": "x.Y", "codec_id": "{codec}"}}"#);
        db.execute("INSERT INTO _streams VALUES (?1, ?2)", (name, config))
            .unwrap();
        db.execute_batch(&format!(
            r#"CREATE TABLE "{name}" (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL);
               CREATE INDEX "{name}_ts" ON "{name}"(ts);
               CREATE TABLE "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL);"#
        ))
        .unwrap();
        for (ts, data) in rows {
            db.execute(&format!(r#"INSERT INTO "{name}" (ts) VALUES (?1)"#), [ts])
                .unwrap();
            let id = db.last_insert_rowid();
            db.execute(
                &format!(r#"INSERT INTO "{name}_blob" VALUES (?1, ?2)"#),
                (id, data),
            )
            .unwrap();
        }
    }
}

/// A dimos-wire `.mcap`, written stream by stream: topic `dimos/<port>/<msg_name>`, lcm-encoded, observation time in log_time.
fn write_mcap(path: &Path, streams: &[(&str, Rows)]) {
    let mut writer = mcap::Writer::new(std::io::BufWriter::new(
        std::fs::File::create(path).unwrap(),
    ))
    .unwrap();
    let mut messages = Vec::new();
    for (name, rows) in streams {
        let id = writer
            .add_channel(
                0,
                &format!("dimos/{name}/std_msgs.String"),
                "lcm",
                &BTreeMap::new(),
            )
            .unwrap();
        messages.extend(rows.iter().map(|(ts, data)| (id, *ts, data.clone())));
    }
    // One channel after another, like db2mcap: file order is not time order.
    for (sequence, (channel_id, ts, data)) in messages.into_iter().enumerate() {
        let ns = (ts * 1e9) as u64;
        let header = mcap::records::MessageHeader {
            channel_id,
            sequence: sequence as u32,
            log_time: ns,
            publish_time: ns,
        };
        writer.write_to_known_channel(&header, &data).unwrap();
    }
    writer.finish().unwrap();
}

fn rows(times: &[f64], tag: u8) -> Rows {
    times
        .iter()
        .enumerate()
        .map(|(i, t)| (T0 + t, vec![tag, i as u8]))
        .collect()
}

fn config(db: &Path, ports: &[&str]) -> ReplayConfig {
    ReplayConfig {
        db: db.display().to_string(),
        speed: 1.0,
        r#loop: false,
        exit_on_complete: false,
        seek: 0.0,
        duration: 0.0,
        loop_gap: 0.05,
        static_period: 1.0,
        ports: ports.iter().map(|p| p.to_string()).collect(),
    }
}

fn lz4(data: &[u8]) -> Vec<u8> {
    let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
    encoder.write_all(data).unwrap();
    encoder.finish().unwrap()
}

/// Runs `play` with a stop handle kept alive, returning `(stream index, payload, wall offset)`.
fn played(
    source: &Source,
    plan: &Plan,
    config: &ReplayConfig,
    limit: usize,
) -> Vec<(usize, Vec<u8>, f64)> {
    let (_keep, stop) = channel();
    let start = Instant::now();
    let mut got = Vec::new();
    play(source, plan, config, &stop, &mut |index, data| {
        got.push((index, data, start.elapsed().as_secs_f64()));
        got.len() < limit
    })
    .unwrap();
    got
}

fn sample_db(dir: &TempDir) -> std::path::PathBuf {
    let path = dir.path().join("rec.db");
    write_db(
        &path,
        &[
            ("odom", "lcm", rows(&[0.0, 0.02, 0.04], 1)),
            ("lidar", "lz4+lcm", vec![(T0 + 0.01, lz4(b"cloud"))]),
            (
                "camera_info",
                "lcm",
                vec![(T0 - 50.0, vec![9]), (T0 - 50.0, vec![9])],
            ),
            ("notes", "pickle", rows(&[0.0], 3)),
            ("empty", "lcm", vec![]),
        ],
    );
    path
}

#[test]
fn plan_splits_timed_and_static_streams() {
    let dir = TempDir::new().unwrap();
    let path = sample_db(&dir);
    let source = Source::open(&path).unwrap();
    let plan = Plan::new(
        &source,
        &config(&path, &["camera_info", "empty", "lidar", "odom"]),
    )
    .unwrap();
    assert_eq!(plan.timed, ["lidar", "odom"]);
    assert_eq!(plan.statics, ["camera_info"]);
    assert_eq!(plan.span, Some((T0, T0 + 0.04)));
    // seek counts from the earliest stream in the file, static ones included, like Replay.first_ts().
    assert_eq!(plan.start, T0 - 50.0);
}

#[test]
fn plan_skips_missing_and_rejects_non_lcm_streams() {
    let dir = TempDir::new().unwrap();
    let path = sample_db(&dir);
    let source = Source::open(&path).unwrap();
    // A subclass may declare a port this recording lacks: it is planned as neither timed nor static.
    let missing = Plan::new(&source, &config(&path, &["nope", "odom"])).unwrap();
    assert_eq!(missing.timed, ["odom"]);
    assert!(missing.statics.is_empty());
    let pickle = Plan::new(&source, &config(&path, &["notes"])).unwrap_err();
    assert!(pickle.to_string().contains("stored as pickle"), "{pickle}");
}

#[test]
fn db_replays_wire_bytes_in_timestamp_order() {
    let dir = TempDir::new().unwrap();
    let path = sample_db(&dir);
    let source = Source::open(&path).unwrap();
    let config = config(&path, &["lidar", "odom"]);
    let plan = Plan::new(&source, &config).unwrap();
    let got = played(&source, &plan, &config, usize::MAX);
    let order: Vec<(usize, Vec<u8>)> = got.iter().map(|(i, d, _)| (*i, d.clone())).collect();
    assert_eq!(
        order,
        [
            (1, vec![1, 0]),
            (0, b"cloud".to_vec()),
            (1, vec![1, 1]),
            (1, vec![1, 2])
        ],
        "lz4 is unwrapped and streams interleave by ts"
    );
    let last = got.last().unwrap().2;
    assert!(
        (0.035..1.0).contains(&last),
        "recorded 40 ms span played in {last}s"
    );
}

#[test]
fn speed_seek_and_duration_bound_the_window() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("rec.db");
    write_db(
        &path,
        &[("odom", "lcm", rows(&[0.0, 0.1, 0.2, 0.3, 0.4], 1))],
    );
    let source = Source::open(&path).unwrap();
    let mut config = config(&path, &["odom"]);
    config.seek = 0.1;
    config.duration = 0.2;
    config.speed = 4.0;
    let plan = Plan::new(&source, &config).unwrap();
    let got = played(&source, &plan, &config, usize::MAX);
    let payloads: Vec<u8> = got.iter().map(|(_, d, _)| d[1]).collect();
    assert_eq!(payloads, [1, 2, 3]);
    let last = got.last().unwrap().2;
    // At 1x it would take 0.2 s; the upper bound leaves room for a loaded machine.
    assert!(
        (0.045..0.2).contains(&last),
        "0.2 s at 4x played in {last}s"
    );
}

#[test]
fn loop_wraps_with_the_gap_until_stopped() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("rec.db");
    write_db(&path, &[("odom", "lcm", rows(&[0.0, 0.02], 1))]);
    let source = Source::open(&path).unwrap();
    let mut config = config(&path, &["odom"]);
    config.r#loop = true;
    let plan = Plan::new(&source, &config).unwrap();
    let got = played(&source, &plan, &config, 5);
    let payloads: Vec<u8> = got.iter().map(|(_, d, _)| d[1]).collect();
    assert_eq!(payloads, [0, 1, 0, 1, 0]);
    // The third message starts pass 1: one span (0.02) plus the gap (0.05) after the first.
    assert!(got[2].2 >= 0.07, "second pass started at {}", got[2].2);
}

#[test]
fn dropping_the_stop_handle_ends_a_wait() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("rec.db");
    write_db(&path, &[("odom", "lcm", rows(&[0.0, 30.0], 1))]);
    let source = Source::open(&path).unwrap();
    let config = config(&path, &["odom"]);
    let plan = Plan::new(&source, &config).unwrap();
    let (stop_tx, stop) = channel::<()>();
    let started = Instant::now();
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(50));
        drop(stop_tx);
    });
    let mut count = 0;
    play(&source, &plan, &config, &stop, &mut |_, _| {
        count += 1;
        true
    })
    .unwrap();
    assert_eq!(count, 1);
    assert!(started.elapsed() < Duration::from_secs(5));
}

#[test]
fn mcap_streams_are_named_by_port_and_replayed() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("rec.mcap");
    write_mcap(
        &path,
        &[
            ("odom", rows(&[0.0, 0.02], 1)),
            ("tf", vec![(T0 + 0.01, vec![7])]),
        ],
    );
    let source = Source::open(&path).unwrap();
    assert_eq!(source.storages().keys().collect::<Vec<_>>(), ["odom", "tf"]);
    let config = config(&path, &["odom", "tf"]);
    let plan = Plan::new(&source, &config).unwrap();
    assert_eq!(plan.span, Some((T0, T0 + 0.02)));
    let order: Vec<(usize, Vec<u8>)> = played(&source, &plan, &config, usize::MAX)
        .into_iter()
        .map(|(i, d, _)| (i, d))
        .collect();
    assert_eq!(order, [(0, vec![1, 0]), (1, vec![7]), (0, vec![1, 1])]);
}

#[test]
fn due_scales_by_speed_and_offsets_each_pass() {
    let span = (10.0, 12.0);
    assert_eq!(due(11.0, 0, span, 2.0, 0.5), Duration::from_secs_f64(0.5));
    assert_eq!(due(10.0, 1, span, 1.0, 0.5), Duration::from_secs_f64(2.5));
    assert_eq!(due(9.0, 0, span, 1.0, 0.5), Duration::ZERO);
}
