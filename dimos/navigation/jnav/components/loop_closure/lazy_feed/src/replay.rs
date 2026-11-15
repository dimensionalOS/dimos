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

//! Replay the MLS planner alone from a lazy-feed file (see main.rs), in order: GSC's new_keyframe and
//! loop_closure_event messages, each scan's version-stamped tracer region, and a replan to a goal near the start
//! every `--replan-every` scans. Writes a deterministic per-scan record and summary, and the timings apart.

use std::collections::BTreeMap;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::Instant;

use dimos_mls_planner::lazy::{decode_cloud, decode_loop_closure_event, decode_new_keyframe, Pose};
use dimos_mls_planner::mls_planner::{Config, MapPoint, Plan, Planner, RegionBounds, VersionGap};
use dimos_mls_planner::replan::{pass_replans, replan, PassTrigger, Replan};

const NEW_KEYFRAME: u8 = 1;
const LOOP_CLOSURE_EVENT: u8 = 2;
const SCAN: u8 = 3;
const END: u8 = 9;

struct Args {
    feed: PathBuf,
    planner_config: PathBuf,
    output: PathBuf,
    goal_along_path_m: f64,
    until_scan: u64,
    dump_dir: Option<PathBuf>,
    dump_radius_m: f32,
    dump_limit: usize,
    /// Snapshots to dump after a scan, by scan index and center.
    snapshot_at: Vec<(u64, f32, f32)>,
    /// Scans after which the planner is checked against a full rebuild from its own voxels.
    parity_at: Vec<u64>,
    /// Inclusive scan ranges left out of path_share_excluded, e.g. where the odometry is known to drift.
    exclude_scans: Vec<(u64, u64)>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        feed: PathBuf::new(),
        planner_config: PathBuf::new(),
        output: PathBuf::new(),
        goal_along_path_m: 2.0,
        until_scan: u64::MAX,
        dump_dir: None,
        dump_radius_m: 40.0,
        dump_limit: 4,
        snapshot_at: Vec::new(),
        parity_at: Vec::new(),
        exclude_scans: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--feed" => args.feed = value()?.into(),
            "--planner-config" => args.planner_config = value()?.into(),
            "--output" => args.output = value()?.into(),
            "--goal-along-path-m" => {
                args.goal_along_path_m = value()?.parse().map_err(|e| format!("{flag}: {e}"))?
            }
            "--until-scan" => args.until_scan = value()?.parse().map_err(|e| format!("{flag}: {e}"))?,
            "--dump-dir" => args.dump_dir = Some(value()?.into()),
            "--dump-radius-m" => args.dump_radius_m = value()?.parse().map_err(|e| format!("{flag}: {e}"))?,
            "--dump-limit" => args.dump_limit = value()?.parse().map_err(|e| format!("{flag}: {e}"))?,
            "--snapshot-at" => {
                let v = value()?;
                let parts: Vec<&str> = v.split(',').collect();
                let bad = || format!("{flag} takes SCAN,X,Y, not {v}");
                if parts.len() != 3 {
                    return Err(bad());
                }
                args.snapshot_at.push((
                    parts[0].parse().map_err(|_| bad())?,
                    parts[1].parse().map_err(|_| bad())?,
                    parts[2].parse().map_err(|_| bad())?,
                ));
            }
            "--exclude-scans" => {
                let v = value()?;
                let bad = || format!("{flag} takes FIRST-LAST, not {v}");
                let (a, b) = v.split_once('-').ok_or_else(bad)?;
                args.exclude_scans.push((a.parse().map_err(|_| bad())?, b.parse().map_err(|_| bad())?));
            }
            "--parity-at" => args.parity_at.push(value()?.parse().map_err(|e| format!("{flag}: {e}"))?),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if args.feed.as_os_str().is_empty()
        || args.planner_config.as_os_str().is_empty()
        || args.output.as_os_str().is_empty()
    {
        return Err("usage: lazy-replay --feed F --planner-config C.json --output DIR \
                    [--goal-along-path-m M] [--until-scan S] [--exclude-scans FIRST-LAST]... [--parity-at SCAN]... [--dump-dir D [--dump-radius-m R] [--dump-limit N] [--snapshot-at SCAN,X,Y]...]"
            .into());
    }
    Ok(args)
}

/// The feed's records, decoded one at a time; a region is kept as the tracer's last full set of rows.
struct Feed<R: Read> {
    stream: R,
    region: BTreeMap<[u32; 3], [f32; 7]>,
}

enum Record {
    Message { tag: u8, lcm: Vec<u8> },
    Scan { index: u64, ts: f64, pose: [f64; 7], version: u32, region: Option<([f32; 5], Vec<MapPoint>)> },
    End(serde_json::Value),
}

fn exact<R: Read>(stream: &mut R, n: usize) -> Result<Vec<u8>, String> {
    let mut buf = vec![0u8; n];
    stream.read_exact(&mut buf).map_err(|e| format!("feed ended mid-record: {e}"))?;
    Ok(buf)
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

fn f32_at(b: &[u8], at: usize) -> f32 {
    f32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

impl<R: Read> Feed<R> {
    fn open(mut stream: R) -> Result<(serde_json::Value, Self), String> {
        if exact(&mut stream, 8)? != b"LAZYFEED" {
            return Err("not a lazy-feed file".into());
        }
        let format = u32_at(&exact(&mut stream, 4)?, 0);
        if format != 2 {
            return Err(format!("feed format {format}, this reader knows 2"));
        }
        let length = u32_at(&exact(&mut stream, 4)?, 0) as usize;
        let header = serde_json::from_slice(&exact(&mut stream, length)?).map_err(|e| e.to_string())?;
        Ok((header, Feed { stream, region: BTreeMap::new() }))
    }

    fn next(&mut self) -> Result<Record, String> {
        let head = exact(&mut self.stream, 17)?;
        let tag = head[0];
        let index = u64::from_le_bytes(head[1..9].try_into().unwrap());
        let ts = f64::from_le_bytes(head[9..17].try_into().unwrap());
        match tag {
            NEW_KEYFRAME | LOOP_CLOSURE_EVENT => {
                let n = u32_at(&exact(&mut self.stream, 4)?, 0) as usize;
                Ok(Record::Message { tag, lcm: exact(&mut self.stream, n)? })
            }
            SCAN => {
                let raw = exact(&mut self.stream, 56)?;
                let pose: [f64; 7] =
                    std::array::from_fn(|i| f64::from_le_bytes(raw[8 * i..8 * i + 8].try_into().unwrap()));
                let tail = exact(&mut self.stream, 5)?;
                let version = u32_at(&tail, 0);
                let region = if tail[4] != 0 {
                    let c = exact(&mut self.stream, 20)?;
                    let cylinder: [f32; 5] = std::array::from_fn(|i| f32_at(&c, 4 * i));
                    let removed = u32_at(&exact(&mut self.stream, 4)?, 0) as usize;
                    let gone = exact(&mut self.stream, removed * 12)?;
                    for k in 0..removed {
                        self.region.remove(&std::array::from_fn(|i| u32_at(&gone, 12 * k + 4 * i)));
                    }
                    let upserted = u32_at(&exact(&mut self.stream, 4)?, 0) as usize;
                    let rows = exact(&mut self.stream, upserted * 28)?;
                    for k in 0..upserted {
                        let row: [f32; 7] = std::array::from_fn(|i| f32_at(&rows, 28 * k + 4 * i));
                        self.region.insert(std::array::from_fn(|i| row[i].to_bits()), row);
                    }
                    let points = self
                        .region
                        .values()
                        .map(|r| ((r[0], r[1], r[2]), [r[3], r[4], r[5], r[6]]))
                        .collect();
                    Some((cylinder, points))
                } else {
                    None
                };
                Ok(Record::Scan { index, ts, pose, version, region })
            }
            END => {
                let mut rest = Vec::new();
                self.stream.read_to_end(&mut rest).map_err(|e| e.to_string())?;
                Ok(Record::End(serde_json::from_slice(&rest).unwrap_or(serde_json::Value::Null)))
            }
            other => Err(format!("unknown record tag {other}")),
        }
    }
}

/// Seconds per call of one stage, for p50/p99/max; per planner stage of update_region and closure, ms per call
/// (0 for a call that skipped it); and each call's breakdown, for the slowest.
#[derive(Default)]
struct Timings {
    calls: BTreeMap<&'static str, Vec<f64>>,
    stages: BTreeMap<&'static str, BTreeMap<String, Vec<f64>>>,
    breakdowns: BTreeMap<&'static str, Vec<(f64, u64, serde_json::Value)>>,
    scan: u64,
}

fn quantiles(samples: &[f64], scale: f64) -> serde_json::Value {
    let mut s = samples.to_vec();
    s.sort_by(f64::total_cmp);
    let at = |q: f64| s[((s.len() - 1) as f64 * q).round() as usize];
    serde_json::json!({
        "calls": s.len(), "total_s": s.iter().sum::<f64>() * scale / 1e3,
        "p50_ms": scale * at(0.5), "p99_ms": scale * at(0.99), "max_ms": scale * at(1.0),
    })
}

impl Timings {
    fn time<T>(&mut self, kind: &'static str, f: impl FnOnce() -> T) -> T {
        let start = Instant::now();
        let out = f();
        self.calls.entry(kind).or_default().push(start.elapsed().as_secs_f64());
        out
    }

    /// Time a planner call and keep its per-stage breakdown.
    fn time_staged<T>(&mut self, kind: &'static str, planner: &mut Planner, f: impl FnOnce(&mut Planner) -> T) -> T {
        let start = Instant::now();
        let out = f(planner);
        let total = start.elapsed().as_secs_f64();
        self.calls.entry(kind).or_default().push(total);
        let calls = self.calls[kind].len();
        let stages = self.stages.entry(kind).or_default();
        let mut seen = std::collections::BTreeSet::new();
        let mut breakdown = serde_json::Map::new();
        for s in planner.last_stage_ms() {
            let name = format!("{}.{}", s.phase, s.stage);
            breakdown.insert(name.clone(), serde_json::json!((s.ms * 1e3).round() / 1e3));
            let samples = stages.entry(name.clone()).or_default();
            // a stage first reached now skipped every earlier call
            samples.resize(calls - 1, 0.0);
            samples.push(s.ms);
            seen.insert(name);
        }
        for (name, samples) in stages.iter_mut() {
            if !seen.contains(name) {
                samples.resize(calls, 0.0);
            }
        }
        let counts: serde_json::Map<String, serde_json::Value> = planner
            .last_stage_counts()
            .iter()
            .map(|(phase, name, n)| (format!("{phase}.{name}"), serde_json::json!(n)))
            .collect();
        let slow = self.breakdowns.entry(kind).or_default();
        slow.push((total, self.scan, serde_json::json!({ "stages_ms": breakdown, "counts": counts })));
        if slow.len() > 200 {
            slow.sort_by(|a, b| b.0.total_cmp(&a.0));
            slow.truncate(SLOWEST);
        }
        out
    }

    fn summary(&self) -> serde_json::Value {
        let mut out = serde_json::Map::new();
        for (kind, samples) in &self.calls {
            let mut entry = quantiles(samples, 1e3);
            if let Some(stages) = self.stages.get(kind) {
                let by_stage: serde_json::Map<String, serde_json::Value> =
                    stages.iter().map(|(name, s)| (name.clone(), quantiles(s, 1.0))).collect();
                entry["stages"] = serde_json::Value::Object(by_stage);
            }
            out.insert(kind.to_string(), entry);
        }
        serde_json::Value::Object(out)
    }

    fn slowest(&self) -> serde_json::Value {
        let mut out = serde_json::Map::new();
        for (kind, calls) in &self.breakdowns {
            let mut calls: Vec<_> = calls.iter().collect();
            calls.sort_by(|a, b| b.0.total_cmp(&a.0));
            let list: Vec<serde_json::Value> = calls
                .iter()
                .take(SLOWEST)
                .map(|(total, scan, detail)| {
                    let mut d = detail.clone();
                    d["scan"] = serde_json::json!(scan);
                    d["total_ms"] = serde_json::json!((total * 1e6).round() / 1e3);
                    d
                })
                .collect();
            out.insert(kind.to_string(), serde_json::Value::Array(list));
        }
        serde_json::Value::Object(out)
    }
}

/// How many of the slowest calls of each kind slow_calls.json keeps.
const SLOWEST: usize = 20;

fn path_length(waypoints: &[(f32, f32, f32)]) -> f64 {
    waypoints
        .windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            (((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2) + (b.2 - a.2).powi(2)) as f64).sqrt()
        })
        .sum()
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let config: Config = serde_json::from_str(
        &std::fs::read_to_string(&args.planner_config).map_err(|e| format!("{:?}: {e}", args.planner_config))?,
    )
    .map_err(|e| format!("{:?}: {e}", args.planner_config))?;
    let file = std::fs::File::open(&args.feed).map_err(|e| format!("{:?}: {e}", args.feed))?;
    let decoder = zstd::stream::read::Decoder::new(file).map_err(|e| e.to_string())?;
    let (header, mut feed) = Feed::open(std::io::BufReader::new(decoder))?;
    std::fs::create_dir_all(&args.output).map_err(|e| e.to_string())?;
    let mut scans_out = BufWriter::new(
        std::fs::File::create(args.output.join("scans.csv")).map_err(|e| e.to_string())?,
    );
    writeln!(scans_out, "scan,ts,version,layers,pass,replanned,path,path_m,seam_check").map_err(|e| e.to_string())?;

    let mut planner = Planner::new(config.worker_threads);
    let mut timings = Timings::default();
    // each keyframe's current pose, kept from new_keyframe and loop_closure_event, which the goal rides
    let mut keyframes: BTreeMap<u32, Pose> = BTreeMap::new();
    let mut goal: Option<(u32, [f64; 3])> = None;
    let mut travelled = 0.0;
    let mut last_xyz: Option<[f64; 3]> = None;
    let mut held: Option<(u64, f64, [f64; 7], u32, ([f32; 5], Vec<MapPoint>))> = None;
    let (mut first_ts, mut last_ts) = (None, 0.0);
    let (mut replans, mut found, mut counted, mut counted_found) = (0u64, 0u64, 0u64, 0u64);
    let (mut kept, mut kept_found) = (0u64, 0u64);
    let (mut events, mut new_layers, mut seam_checks, mut torn) = (0u64, 0u64, 0u64, 0u64);
    let mut pending_seam_check = false;
    let mut last_found = false;
    // the module's state between worker passes: a closure that started a layer, the route it follows
    let mut closure_pending = false;
    let mut route: Option<(f64, bool)> = None;
    let mut planned_once = false;
    let mut passes = 0u64;
    let mut robot_xy = (0.0f32, 0.0f32);
    let mut last_cylinder: Option<(f32, f32, f32)> = None;
    // a closure that started a layer while paths were found: the snapshot from just before it, kept until the next replan
    let mut before_seam: Option<(String, String)> = None;
    let mut dumps = 0usize;
    let dump = |name: &str, json: &str| {
        if let Some(dir) = &args.dump_dir {
            let _ = std::fs::create_dir_all(dir);
            let _ = std::fs::write(dir.join(format!("{name}.json")), json);
        }
    };
    let wall = Instant::now();

    let apply_region = |planner: &mut Planner,
                        timings: &mut Timings,
                        pose: &[f64; 7],
                        version: u32,
                        region: &([f32; 5], Vec<MapPoint>)|
     -> Result<bool, VersionGap> {
        let (c, points) = region;
        let bounds = RegionBounds { origin_x: c[0], origin_y: c[1], radius: c[2], z_min: c[3], z_max: c[4] }
            .capped_at(pose[2] as f32, config.max_overhead_m);
        timings.time_staged("update_region", planner, |p| p.update_region_at_version(points, &bounds, version, &config))?;
        Ok(true)
    };

    let summary_in = loop {
        match feed.next()? {
            Record::Message { tag, lcm } => {
                let cloud = decode_cloud(&lcm).map_err(|e| e.0)?;
                if tag == NEW_KEYFRAME {
                    let records = decode_new_keyframe(&cloud).map_err(|e| e.0)?;
                    for r in &records {
                        keyframes.insert(r.node_id, r.raw);
                    }
                    planner.add_keyframes(&records, &config);
                } else {
                    let records = decode_loop_closure_event(&cloud).map_err(|e| e.0)?;
                    for r in &records {
                        keyframes.insert(r.node_id, r.post());
                    }
                    events += 1;
                    let snapshot = (args.dump_dir.is_some() && dumps < args.dump_limit && last_found).then(|| {
                        let cyl: Vec<_> = last_cylinder.into_iter().collect();
                        planner.snapshot_json("before", robot_xy, args.dump_radius_m, &cyl, &[])
                    });
                    let outcome = timings.time_staged("closure", &mut planner, |p| p.on_closure(&records, &config));
                    if outcome.as_ref().is_some_and(|o| pass_replans(PassTrigger::Closure { new_layer: o.new_layer })) {
                        closure_pending = true;
                    }
                    if outcome.is_some_and(|o| o.new_layer) {
                        new_layers += 1;
                        pending_seam_check = last_found;
                        if let Some(before) = snapshot {
                            before_seam = Some((format!("v{}", planner.reform_version()), before));
                        }
                    }
                    // a cloud held for this version can go in now
                    if let Some((_, _, pose, version, region)) = held.take() {
                        if let Err(VersionGap::Ahead) = apply_region(&mut planner, &mut timings, &pose, version, &region)
                        {
                            held = Some((0, 0.0, pose, version, region));
                        }
                    }
                }
            }
            Record::Scan { index, ts, pose, version, region } => {
                if index >= args.until_scan {
                    break serde_json::Value::Null;
                }
                timings.scan = index;
                first_ts.get_or_insert(ts);
                last_ts = ts;
                let mut live_update_applied = false;
                if let Some(region) = region {
                    last_cylinder = Some((region.0[0], region.0[1], region.0[2]));
                    match apply_region(&mut planner, &mut timings, &pose, version, &region) {
                        Err(VersionGap::Ahead) => held = Some((index, ts, pose, version, region)),
                        Err(VersionGap::Forgotten) => {}
                        Ok(_) => live_update_applied = true,
                    }
                }
                for &(_, x, y) in args.snapshot_at.iter().filter(|s| s.0 == index) {
                    let cyl: Vec<_> = last_cylinder.into_iter().collect();
                    dump(&format!("at{index}"), &planner.snapshot_json(&format!("scan {index}"), (x, y), args.dump_radius_m, &cyl, &[]));
                }
                let xyz = [pose[0], pose[1], pose[2]];
                robot_xy = (pose[0] as f32, pose[1] as f32);
                if let Some(prev) = last_xyz {
                    travelled += ((xyz[0] - prev[0]).powi(2) + (xyz[1] - prev[1]).powi(2)).sqrt();
                }
                last_xyz = Some(xyz);
                // the goal: where the robot is once it has gone goal_along_path_m, held by the newest keyframe's frame
                if goal.is_none() && travelled >= args.goal_along_path_m {
                    if let Some((&node, kf)) = keyframes.iter().next_back() {
                        goal = Some((node, kf.inverse().apply([xyz[0], xyz[1], xyz[2] - config.start_z_offset_m as f64])));
                    }
                }
                let mut replanned = false;
                let mut pass = false;
                let mut path_m = f64::NAN;
                let mut has_path = false;
                let mut seam_check = false;
                if let Some((node, local)) = goal {
                    // as the module's worker: a pass plans after a closure that started a layer, a goal change or an applied live update
                    let goal_changed = !planned_once;
                    if closure_pending || pass_replans(PassTrigger::Update { goal_changed, live_update_applied }) {
                        closure_pending = false;
                        pass = true;
                        passes += 1;
                        let g = keyframes[&node].apply(local);
                        let goal_xyz = (g[0] as f32, g[1] as f32, g[2] as f32);
                        let start = (pose[0] as f32, pose[1] as f32, (pose[2] - config.start_z_offset_m as f64) as f32);
                        let decided = timings.time("plan_pass", || replan(&mut planner, start, goal_xyz, goal_changed, &config));
                        planned_once = true;
                        match &decided {
                            Replan::Planned(plan) => {
                                replanned = true;
                                replans += 1;
                                route = match plan {
                                    Plan::Full(w) => Some((path_length(w), true)),
                                    _ => None,
                                };
                            }
                            Replan::Kept => {}
                            Replan::Reached => route = Some((0.0, true)),
                        }
                        has_path = route.is_some();
                        path_m = route.map_or(f64::NAN, |r| r.0);
                        if has_path {
                            found += 1;
                        }
                        if pending_seam_check {
                            seam_check = true;
                            seam_checks += 1;
                            torn += u64::from(!has_path);
                            pending_seam_check = false;
                            if let Some((tag, before)) = before_seam.take() {
                                if !has_path && dumps < args.dump_limit {
                                    dumps += 1;
                                    let cyl: Vec<_> = last_cylinder.into_iter().collect();
                                    let gxy = (goal_xyz.0, goal_xyz.1);
                                    dump(&format!("seam{index}_{tag}_0before"), &before.replace("\"before\"", &format!("\"seam{index} before\"")));
                                    dump(&format!("seam{index}_{tag}_1after"), &planner.snapshot_json(&format!("seam{index} after"), robot_xy, args.dump_radius_m, &cyl, &[]));
                                    eprintln!("seam{index} connectivity: {}", planner.connectivity_json(start, goal_xyz, &config));
                                    dump(&format!("seam{index}_{tag}_2goal"), &planner.snapshot_json(&format!("seam{index} at goal"), gxy, args.dump_radius_m, &[], &[]));
                                }
                            }
                        }
                        if last_found && !has_path && !seam_check && dumps < args.dump_limit {
                            dumps += 1;
                            eprintln!("lost path at scan {index} connectivity: {}", planner.connectivity_json(start, goal_xyz, &config));
                            dump(&format!("lost{index}"), &planner.snapshot_json(&format!("lost path {index}"), robot_xy, args.dump_radius_m, &[], &[]));
                        }
                        if args.parity_at.contains(&index) {
                            let fresh = planner.rebuilt(&config);
                            let (only_here, only_fresh) = planner.surface_difference(&fresh);
                            let near = |k: &&(i32, i32, i32)| {
                                let (x, y) = ((k.0 as f32 + 0.5) * config.voxel_size, (k.1 as f32 + 0.5) * config.voxel_size);
                                (x - start.0).hypot(y - start.1) < 5.0
                            };
                            let near_here: Vec<_> = only_here.iter().filter(near).collect();
                            let near_fresh: Vec<_> = only_fresh.iter().filter(near).collect();
                            eprintln!(
                                "parity at scan {index}: start {start:?} goal {goal_xyz:?} has_path {has_path}; cells only incremental {} ({} within 5 m: {:?}), only rebuilt {} ({} within 5 m: {:?})",
                                only_here.len(), near_here.len(), &near_here[..near_here.len().min(40)],
                                only_fresh.len(), near_fresh.len(), &near_fresh[..near_fresh.len().min(40)]
                            );
                            eprintln!("parity at scan {index}: rebuilt plan found {}", fresh.plan(start, goal_xyz, &config).is_some());
                            eprintln!("parity at scan {index}: incremental connectivity {}", planner.connectivity_json(start, goal_xyz, &config));
                            eprintln!("parity at scan {index}: rebuilt connectivity {}", fresh.connectivity_json(start, goal_xyz, &config));
                        }
                        last_found = has_path;
                    } else {
                        has_path = route.is_some();
                        path_m = route.map_or(f64::NAN, |r| r.0);
                    }
                    if found > 0 {
                        counted += 1;
                        counted_found += u64::from(has_path);
                        if !args.exclude_scans.iter().any(|&(a, b)| (a..=b).contains(&index)) {
                            kept += 1;
                            kept_found += u64::from(has_path);
                        }
                    }
                }
                writeln!(
                    scans_out,
                    "{index},{ts:.6},{},{},{},{},{},{:.3},{}",
                    planner.reform_version(),
                    planner.layer_count(),
                    u8::from(pass),
                    u8::from(replanned),
                    u8::from(has_path),
                    path_m,
                    u8::from(seam_check)
                )
                .map_err(|e| e.to_string())?;
            }
            Record::End(summary) => break summary,
        }
    };
    scans_out.flush().map_err(|e| e.to_string())?;
    let data_s = last_ts - first_ts.unwrap_or(last_ts);
    let wall_s = wall.elapsed().as_secs_f64();
    let summary = serde_json::json!({
        "feed": args.feed, "feed_header": header, "feed_summary": summary_in,
        "goal_along_path_m": args.goal_along_path_m, "plan_passes": passes,
        "data_s": data_s, "replans": replans, "replans_with_path": found,
        "replans_counted": counted, "counted_with_path": counted_found,
        "path_share": if counted > 0 { counted_found as f64 / counted as f64 } else { f64::NAN },
        "excluded_scans": args.exclude_scans, "replans_counted_excluded": kept, "counted_with_path_excluded": kept_found,
        "path_share_excluded": if kept > 0 { kept_found as f64 / kept as f64 } else { f64::NAN },
        "events": events, "new_layers": new_layers, "layers": planner.layer_count(),
        "seam_checks": seam_checks, "torn_seams": torn,
    });
    std::fs::write(args.output.join("summary.json"), serde_json::to_string_pretty(&summary).unwrap())
        .map_err(|e| e.to_string())?;
    let timing = serde_json::json!({ "wall_s": wall_s, "x_real_time": data_s / wall_s, "stages": timings.summary() });
    std::fs::write(args.output.join("timings.json"), serde_json::to_string_pretty(&timing).unwrap())
        .map_err(|e| e.to_string())?;
    std::fs::write(args.output.join("slow_calls.json"), serde_json::to_string_pretty(&timings.slowest()).unwrap())
        .map_err(|e| e.to_string())?;
    println!("{}", serde_json::to_string(&summary).unwrap());
    println!("{}", serde_json::to_string(&timing).unwrap());
    Ok(())
}

fn main() {
    tracing_subscriber::fmt().with_env_filter(tracing_subscriber::EnvFilter::from_default_env()).init();
    if let Err(e) = run() {
        eprintln!("lazy-replay: {e}");
        std::process::exit(1);
    }
}
