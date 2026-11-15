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

//! Replay a recording through GSC (its deterministic mode) and the ray tracer in lockstep, one scan at a time with
//! no drops, and record what the MLS planner would receive: every `new_keyframe` and `loop_closure_event` GSC
//! publishes, and per scan the corrected pose plus, when the tracer emits one, its region cylinder and surfaces
//! stamped with the closure version the map is in. The planner can then be replayed alone from the file.
//!
//! File (zstd stream, little-endian): b"LAZYFEED", u32 format, u32 header length, header JSON, then records
//! `u8 tag, u64 scan, f64 ts, payload` (see `Tag`), ending with `Tag::End` and a summary JSON.

mod memory2;

use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config as PgoConfig, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::pointcloud;
use dimos_gsc_pgo::records::GraphOutputs;
use dimos_voxel_ray_tracing::mapper::{Mapper, Pose};
use dimos_voxel_ray_tracing::reform::{decode_loop_closure_event, ReformState};
use dimos_voxel_ray_tracing::voxel_ray_tracer::Config as TracerConfig;
use memory2::{for_each_scan, read_odometry, Rigid, TfTree};
use rusqlite::Connection;

const FORMAT: u32 = 2;

#[repr(u8)]
enum Tag {
    /// GSC's `new_keyframe` message, LCM-encoded PointCloud2.
    NewKeyframe = 1,
    /// GSC's `loop_closure_event` message, LCM-encoded PointCloud2.
    LoopClosureEvent = 2,
    /// The scan's corrected body pose (x y z qx qy qz qw, f64), its map version (u32) and whether a region follows
    /// (u8); a region is cx cy radius z_min z_max (f32), then its surfaces as a change from the previous region's:
    /// u32 count of removed centers (3 f32 each), u32 count of added or changed rows (7 f32: center, unit normal,
    /// plane offset; NaN normal without a plane), as the tracer's `local_map` carries. Rows are keyed by center.
    Scan = 3,
    /// Summary JSON.
    End = 9,
}

struct Args {
    db: PathBuf,
    lidar_stream: String,
    odom_stream: String,
    body_frame: String,
    gsc_config: Option<PathBuf>,
    tracer_config: PathBuf,
    output: PathBuf,
    until_s: f64,
    zstd_level: i32,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        db: PathBuf::new(),
        lidar_stream: String::new(),
        odom_stream: String::new(),
        body_frame: "base_link".into(),
        gsc_config: None,
        tracer_config: PathBuf::new(),
        output: PathBuf::new(),
        until_s: f64::INFINITY,
        zstd_level: 3,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--db" => args.db = value()?.into(),
            "--lidar-stream" => args.lidar_stream = value()?,
            "--odom-stream" => args.odom_stream = value()?,
            "--body-frame" => args.body_frame = value()?,
            "--gsc-config" => args.gsc_config = Some(value()?.into()),
            "--tracer-config" => args.tracer_config = value()?.into(),
            "--output" => args.output = value()?.into(),
            "--until-s" => args.until_s = value()?.parse().map_err(|e| format!("--until-s: {e}"))?,
            "--zstd-level" => args.zstd_level = value()?.parse().map_err(|e| format!("--zstd-level: {e}"))?,
            other => return Err(format!("unknown flag {other}")),
        }
    }
    if args.db.as_os_str().is_empty()
        || args.lidar_stream.is_empty()
        || args.odom_stream.is_empty()
        || args.tracer_config.as_os_str().is_empty()
        || args.output.as_os_str().is_empty()
    {
        return Err("usage: lazy-feed --db <db> --lidar-stream <s> --odom-stream <s> --tracer-config <json> \
                    --output <file> [--gsc-config <json overrides>] [--body-frame base_link] [--until-s s]"
            .into());
    }
    Ok(args)
}

/// GSC's defaults (`Config::default`, from defaults.json) with `overrides` merged over them.
fn gsc_config(overrides: Option<&PathBuf>) -> Result<(PgoConfig, serde_json::Value), String> {
    let mut merged = serde_json::to_value(PgoConfig::default()).map_err(|e| format!("GSC defaults: {e}"))?;
    if let Some(path) = overrides {
        let text = std::fs::read_to_string(path).map_err(|e| format!("read {path:?}: {e}"))?;
        let extra: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{path:?}: {e}"))?;
        let (Some(base), Some(extra)) = (merged.as_object_mut(), extra.as_object()) else {
            return Err("GSC config must be a JSON object".into());
        };
        for (key, value) in extra {
            base.insert(key.clone(), value.clone());
        }
    }
    let config: PgoConfig = serde_json::from_value(merged.clone()).map_err(|e| format!("GSC config: {e}"))?;
    config.validate()?;
    Ok((config, merged))
}

type CenterKey = [u32; 3];
type Region = std::collections::BTreeMap<CenterKey, [f32; 7]>;

/// The tracer's 7-float surface rows keyed by center bits, in a fixed order (its map iterates in hash order).
fn region_rows(flat: &[f32]) -> Region {
    flat.chunks_exact(7)
        .map(|r| ([r[0].to_bits(), r[1].to_bits(), r[2].to_bits()], [r[0], r[1], r[2], r[3], r[4], r[5], r[6]]))
        .collect()
}

/// `next` as a change from `previous`: removed centers, then added or changed rows.
fn put_region_change(out: &mut Vec<u8>, previous: &Region, next: &Region) {
    let removed: Vec<&CenterKey> = previous.keys().filter(|k| !next.contains_key(*k)).collect();
    let upserted: Vec<&[f32; 7]> = next
        .iter()
        .filter(|(k, row)| previous.get(*k).is_none_or(|old| old.map(f32::to_bits) != row.map(f32::to_bits)))
        .map(|(_, row)| row)
        .collect();
    out.extend_from_slice(&(removed.len() as u32).to_le_bytes());
    for key in removed {
        for bits in key {
            out.extend_from_slice(&bits.to_le_bytes());
        }
    }
    out.extend_from_slice(&(upserted.len() as u32).to_le_bytes());
    for row in upserted {
        for v in row {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
}

fn put_pose(out: &mut Vec<u8>, rotation: &mat3::Mat3, translation: &mat3::Vec3) {
    let [qw, qx, qy, qz] = mat3::quat_from_mat(rotation);
    for v in [translation[0], translation[1], translation[2], qx, qy, qz, qw] {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

struct Feed {
    out: zstd::stream::write::Encoder<'static, std::fs::File>,
    sha: sha1_smol::Sha1,
    bytes: u64,
}

impl Feed {
    fn write(&mut self, bytes: &[u8]) -> Result<(), String> {
        self.sha.update(bytes);
        self.bytes += bytes.len() as u64;
        self.out.write_all(bytes).map_err(|e| format!("write feed: {e}"))
    }

    fn record(&mut self, tag: Tag, scan: u64, ts: f64, payload: &[u8]) -> Result<(), String> {
        let mut head = Vec::with_capacity(17);
        head.push(tag as u8);
        head.extend_from_slice(&scan.to_le_bytes());
        head.extend_from_slice(&ts.to_le_bytes());
        self.write(&head)?;
        self.write(payload)
    }

    fn message(&mut self, tag: Tag, scan: u64, ts: f64, encoded: &[u8]) -> Result<(), String> {
        let mut payload = Vec::with_capacity(4 + encoded.len());
        payload.extend_from_slice(&(encoded.len() as u32).to_le_bytes());
        payload.extend_from_slice(encoded);
        self.record(tag, scan, ts, &payload)
    }
}

#[derive(Default)]
struct Counts {
    keyframes: usize,
    events: usize,
    warps: usize,
    /// The tracer map's closure version: events applied so far.
    version: u32,
}

/// GSC's messages for this snapshot into the feed; a closure event also warps the tracer's map, synchronously.
#[allow(clippy::too_many_arguments)]
fn emit(
    pgo: &GscPgo,
    outputs: &mut GraphOutputs,
    mapper: &mut Mapper,
    reform: &mut ReformState,
    tracer_config: &TracerConfig,
    feed: &mut Feed,
    counts: &mut Counts,
    closure_closed: bool,
    scan: u64,
    ts: f64,
) -> Result<(), String> {
    let messages = outputs.step(pgo.key_poses(), pgo.committed_closures(), closure_closed, false, ts, "map");
    if let Some(records) = &messages.new_keyframe {
        counts.keyframes += records.width as usize;
        feed.message(Tag::NewKeyframe, scan, ts, &records.encode())?;
    }
    if let Some((records, _)) = &messages.loop_closure_event {
        counts.events += 1;
        counts.version += 1;
        feed.message(Tag::LoopClosureEvent, scan, ts, &records.encode())?;
        let moved = decode_loop_closure_event(records).map_err(|e| format!("decode event: {e}"))?;
        if let Some(warp) = reform.closure(&moved, &tracer_config.reform) {
            mapper.apply_warp(&warp);
            counts.warps += 1;
        }
    }
    Ok(())
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let (pgo_config, gsc_json) = gsc_config(args.gsc_config.as_ref())?;
    let tracer_json: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&args.tracer_config).map_err(|e| format!("read {:?}: {e}", args.tracer_config))?,
    )
    .map_err(|e| format!("{:?}: {e}", args.tracer_config))?;
    let tracer_config: TracerConfig =
        serde_json::from_value(tracer_json.clone()).map_err(|e| format!("tracer config: {e}"))?;
    if !tracer_config.reform.enabled {
        return Err("the tracer config must have reform.enabled, as the reform blueprint runs it".into());
    }

    let connection = Connection::open_with_flags(&args.db, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
        .map_err(|e| format!("open {:?}: {e}", args.db))?;
    let mut odoms = read_odometry(&connection, &args.odom_stream)?;
    let odom_child = odoms.first().map(|row| row.child_frame_id.clone()).unwrap_or_default();
    let tf_tree = TfTree::read(&connection)?;
    // the body's pose from the odometry child's (odom->body = odom->child * child->body); PoseStamped (no child) is the body
    let child_to_body = if odom_child.is_empty() {
        Rigid::identity()
    } else {
        tf_tree
            .lookup(&odom_child, &args.body_frame)
            .map_err(|e| format!("odometry child {odom_child:?} -> {}: {e}", args.body_frame))?
    };
    for row in &mut odoms {
        row.translation = mat3::add(&row.translation, &mat3::mat_vec(&row.rotation, &child_to_body.translation));
        row.rotation = mat3::mat_mul(&row.rotation, &child_to_body.rotation);
    }

    let worker_threads = gsc_json.get("worker_threads").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
    if let Err(e) = rayon::ThreadPoolBuilder::new().num_threads(worker_threads).build_global() {
        eprintln!("lazy-feed: could not cap the GSC worker threads: {e}");
    }
    let mut pgo = GscPgo::new(pgo_config);
    let mut outputs = GraphOutputs::default();
    let mut mapper = Mapper::new(tracer_config.clone());
    mapper.track_voxel_times();
    let mut reform = ReformState::default();

    let header = serde_json::json!({
        "format": FORMAT,
        "db": args.db,
        "lidar_stream": args.lidar_stream,
        "odom_stream": args.odom_stream,
        "body_frame": args.body_frame,
        "until_s": if args.until_s.is_finite() { serde_json::json!(args.until_s) } else { serde_json::Value::Null },
        "gsc_config": gsc_json,
        "tracer_config": tracer_json,
        "git_head": std::env::var("LAZY_FEED_GIT_HEAD").unwrap_or_default(),
    });
    let header_bytes = serde_json::to_vec(&header).map_err(|e| e.to_string())?;
    let partial = args.output.with_extension("partial");
    let file = std::fs::File::create(&partial).map_err(|e| format!("create {partial:?}: {e}"))?;
    let mut feed = Feed {
        out: zstd::stream::write::Encoder::new(file, args.zstd_level).map_err(|e| e.to_string())?,
        sha: sha1_smol::Sha1::new(),
        bytes: 0,
    };
    feed.write(b"LAZYFEED")?;
    feed.write(&FORMAT.to_le_bytes())?;
    feed.write(&(header_bytes.len() as u32).to_le_bytes())?;
    feed.write(&header_bytes)?;

    let started = Instant::now();
    let mut lidar_to_body: Option<(String, Rigid)> = None;
    let mut cursor = 0usize;
    let mut scan_index = 0u64;
    let mut counts = Counts::default();
    let mut regions = 0usize;
    let mut last_region = Region::new();
    let (mut first_ts, mut last_ts) = (f64::NAN, f64::NAN);

    for_each_scan(&connection, &args.lidar_stream, args.until_s, |scan| {
        let to_body = match &lidar_to_body {
            Some((frame, rigid)) if *frame == scan.frame_id => *rigid,
            Some((frame, _)) => {
                return Err(format!("lidar stream mixes frames {frame:?} and {:?}", scan.frame_id));
            }
            None => {
                let rigid = if odom_child.is_empty() {
                    Rigid::identity()
                } else {
                    tf_tree
                        .lookup(&args.body_frame, &scan.frame_id)
                        .map_err(|e| format!("lidar frame -> {}: {e}", args.body_frame))?
                };
                lidar_to_body = Some((scan.frame_id.clone(), rigid));
                rigid
            }
        };
        // the newest odometry at or before the scan, as the module pairs them
        while cursor + 1 < odoms.len() && odoms[cursor + 1].ts <= scan.ts {
            cursor += 1;
        }
        if odoms.is_empty() || odoms[cursor].ts > scan.ts {
            return Ok(());
        }
        let odom = &odoms[cursor];
        if first_ts.is_nan() {
            first_ts = scan.ts;
        }
        last_ts = scan.ts;
        let mut cloud = scan.points;
        if !to_body.is_identity() {
            cloud = pointcloud::transform_cloud(&cloud, &to_body.rotation, &to_body.translation);
        }
        let mut pose = PoseWithTime::new(odom.rotation, odom.translation);
        let seconds = odom.ts.floor();
        pose.set_time(seconds as i32, ((odom.ts - seconds) * 1e9) as u32);
        let cloud = Arc::new(cloud);
        let cloud_with_pose = CloudWithPose { cloud: Some(Arc::clone(&cloud)), pose };

        // as the module's worker loop
        let keyframe = pgo.add_key_pose(&cloud_with_pose);
        if !keyframe && pgo.adopted_since_update() && pgo.update() {
            emit(&pgo, &mut outputs, &mut mapper, &mut reform, &tracer_config, &mut feed, &mut counts, true, scan_index, scan.ts)?;
        }
        if keyframe {
            pgo.step_closure_search();
            let moved = pgo.update();
            emit(&pgo, &mut outputs, &mut mapper, &mut reform, &tracer_config, &mut feed, &mut counts, moved, scan_index, scan.ts)?;
        }

        // the tracer places the scan by GSC's correction in force now, as tf map -> odom carries it
        let rotation = mat3::mat_mul(&pgo.rotation_offset(), &odom.rotation);
        let translation = mat3::add(&mat3::mat_vec(&pgo.rotation_offset(), &odom.translation), &pgo.translation_offset());
        let [qw, qx, qy, qz] = mat3::quat_from_mat(&rotation);
        let tracer_pose = Pose {
            position: (translation[0] as f32, translation[1] as f32, translation[2] as f32),
            orientation: (qx as f32, qy as f32, qz as f32, qw as f32),
        };
        mapper.set_clock(scan.ts);
        mapper.add_frame(cloud.iter().map(|p| (p[0], p[1], p[2])).collect(), tracer_pose);
        let region = mapper.local_due().then(|| mapper.take_local_bounds());

        let mut payload = Vec::new();
        put_pose(&mut payload, &rotation, &translation);
        payload.extend_from_slice(&counts.version.to_le_bytes());
        match region {
            Some(cylinder) => {
                regions += 1;
                let rows = region_rows(&mapper.local_surfaces(&cylinder.bounds()));
                payload.push(1);
                for v in [cylinder.cx, cylinder.cy, cylinder.radius, cylinder.z_min, cylinder.z_max] {
                    payload.extend_from_slice(&v.to_le_bytes());
                }
                put_region_change(&mut payload, &last_region, &rows);
                last_region = rows;
            }
            None => payload.push(0),
        }
        feed.record(Tag::Scan, scan_index, scan.ts, &payload)?;
        scan_index += 1;
        if scan_index % 2000 == 0 {
            eprintln!(
                "lazy-feed: {scan_index} scans, {:.0} s of data, {} keyframes, {} events, {:.0} s wall",
                scan.ts - first_ts,
                pgo.key_poses().len(),
                counts.events,
                started.elapsed().as_secs_f64()
            );
        }
        Ok(())
    })?;
    // end of data: land every outstanding job, as the module does on stop
    if pgo.finish() {
        emit(&pgo, &mut outputs, &mut mapper, &mut reform, &tracer_config, &mut feed, &mut counts, true, scan_index, last_ts)?;
    }
    let wall_s = started.elapsed().as_secs_f64();
    let summary = serde_json::json!({
        "scans": scan_index,
        "keyframes": counts.keyframes,
        "loop_closure_events": counts.events,
        "versions": counts.version,
        "tracer_warps": counts.warps,
        "regions": regions,
        "committed_closures": pgo.committed_closures().len(),
        "data_s": last_ts - first_ts,
    });
    // the file holds only what the data determines; timings go to stdout
    let summary_bytes = serde_json::to_vec(&summary).map_err(|e| e.to_string())?;
    feed.record(Tag::End, scan_index, last_ts, &summary_bytes)?;
    let digest = feed.sha.digest().to_string();
    feed.out.finish().map_err(|e| format!("finish feed: {e}"))?;
    std::fs::rename(&partial, &args.output).map_err(|e| format!("rename to {:?}: {e}", args.output))?;
    println!(
        "{}",
        serde_json::json!({
            "summary": summary,
            "content_sha1": digest,
            "raw_bytes": feed.bytes,
            "wall_s": wall_s,
            "x_real": (last_ts - first_ts) / wall_s,
        })
    );
    Ok(())
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "warn".into()))
        .with_writer(std::io::stderr)
        .init();
    if let Err(e) = run() {
        eprintln!("lazy-feed: {e}");
        std::process::exit(1);
    }
}
