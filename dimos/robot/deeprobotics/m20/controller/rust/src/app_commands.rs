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

//! basic_server's command table (CmdHandle::init 0x1863a8) into the operator's command path, replies at once.

use std::net::SocketAddr;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use serde::Serialize;
use serde_json::Value;
use tracing::{info, warn};

use crate::app_frame::{now_string, Frame, FORMAT_JSON};
use crate::app_reports::{self, Snapshots};
use crate::app_server::Clients;
use crate::basic_server::{extre_gait, known_gait, UsageMode};
use crate::config::AppServerConfig;
use crate::dock::docking::{DockFeed, Request};
use crate::operator::Operator;
use crate::types::Target;

/// gs_CmdErrorMsg (CmdHandle's static map): each code's ErrorMessage.
pub mod error {
    pub const OK: i32 = 0;
    pub const UNSUPPORTED_FORMAT: i32 = 0xe001;
    pub const PARSE_FAILED: i32 = 0xe002;
    pub const UNSUPPORTED_TYPE: i32 = 0xe003;
    pub const MISSING_FIELD: i32 = 0xe004;
    pub const MISMATCHED_TYPE: i32 = 0xe005;
    pub const MISMATCHED_CLIENT: i32 = 0xe006;
    pub const NO_PERMISSION: i32 = 0xe007;
    pub const NOT_ALLOWED: i32 = 0xe008;
    pub const FAILED: i32 = 0xe009;
    pub const UNSUPPORTED_FUNCTION: i32 = 0xe00a;
    pub const INTERNAL: i32 = 0xe00b;

    pub fn message(code: i32) -> Option<&'static str> {
        Some(match code {
            UNSUPPORTED_FORMAT => "Unsupported data format",
            PARSE_FAILED => "Data parsing failed",
            UNSUPPORTED_TYPE => "Unsupported protocol type",
            MISSING_FIELD => "Missed necessary feild",
            MISMATCHED_TYPE => "Mismatched data type",
            MISMATCHED_CLIENT => "Mismatched request client",
            NO_PERMISSION => "No operation permission",
            NOT_ALLOWED => "Not allowed operation",
            FAILED => "Operation failure",
            UNSUPPORTED_FUNCTION => "Unsupported function",
            INTERNAL => "Internal error",
            _ => return None,
        })
    }
}

/// TODO(jeff): decide each; until then basic_server's "Unsupported function" (0xe00a).
pub const UNSUPPORTED: [(i64, i64); 12] = [
    (1101, 2), // LED: the m20-ctrlmcu crate could drive it
    (1101, 3), // SetDevice power rails: m20-ctrlmcu
    (1101, 6), // SetSleep: ctrlmcu power rails plus a lie-down / wake in the controller
    (1101, 7), // GetSleepConfig
    (1101, 8), // ResetJointsZero: AutoSetHome needs the motor bus the controller owns
    (1003, 1), // NavTasks: dimos navigation instead (0xe00a is basic_server's own no-nav answer)
    (1003, 2), // NavTasks
    (1004, 1), // CancelNavTask
    (1007, 1), // GetNavTaskStatus
    (1007, 2), // GetLocationStatus
    (2101, 1), // InitLocation
    (2002, 1), // GetNavExecStatus
];

/// What the commands reach.
#[derive(Clone)]
pub struct Links {
    pub operator: Arc<Operator>,
    pub usage: Arc<UsageMode>,
    pub dock: Arc<DockFeed>,
    pub snapshots: Arc<Snapshots>,
    /// the module's cmd_vel_limits (|vx|, |vy|, |wz|): RealSteer meets cmd_vel's clamp (basic_server has none)
    pub velocity_limits: [f64; 3],
}

pub struct App {
    cfg: AppServerConfig,
    links: Links,
}

/// JSONRootToBroker's document: Items first, then Type, Command and Time.
#[derive(Serialize)]
struct Root<'a, T: Serialize> {
    #[serde(rename = "PatrolDevice")]
    device: Device<'a, T>,
}

#[derive(Serialize)]
struct Device<'a, T: Serialize> {
    #[serde(rename = "Items")]
    items: &'a T,
    #[serde(rename = "Type")]
    kind: i64,
    #[serde(rename = "Command")]
    command: i64,
    #[serde(rename = "Time")]
    time: String,
}

/// A PatrolDevice document's text.
pub fn document<T: Serialize>(kind: i64, command: i64, items: &T) -> Vec<u8> {
    let root = Root {
        device: Device {
            items,
            kind,
            command,
            time: now_string(),
        },
    };
    serde_json::to_vec(&root).unwrap_or_default()
}

/// What a handler answers.
enum Answer {
    /// Items {ErrorCode, ErrorMessage when the code has one}
    Code(i32),
    /// these Items as they are (GetSn, GetReleaseNote: no ErrorCode)
    Items(Value),
    /// nothing (a steer taken)
    Silent,
}

/// An Items integer field (a string goes through std::stoi): missing is 0xe004, anything else reads 0.
fn integer(items: &Value, key: &str) -> Result<i64, i32> {
    match items.get(key) {
        None => Err(error::MISSING_FIELD),
        Some(Value::String(t)) => Ok(stoi(t).unwrap_or(0)),
        Some(v) => Ok(v.as_i64().unwrap_or(0)),
    }
}

/// std::stoi: leading whitespace, a sign and digits; the rest ignored.
fn stoi(text: &str) -> Option<i64> {
    let t = text.trim_start();
    let digits = t
        .char_indices()
        .take_while(|&(i, c)| c.is_ascii_digit() || (i == 0 && (c == '-' || c == '+')))
        .count();
    t[..digits].parse().ok()
}

fn code(result: Result<(), i32>) -> Answer {
    Answer::Code(result.err().unwrap_or(error::OK))
}

impl App {
    pub fn new(cfg: AppServerConfig, links: Links) -> App {
        App { cfg, links }
    }

    /// One frame from `from`; the reply, if any (ParseData's, with its message id, client class and format).
    pub fn handle(&self, frame: &Frame, from: SocketAddr, clients: &Clients) -> Option<Frame> {
        clients.activity();
        let plain = |text: &str| Some(Frame::answering(frame, frame.message_id, text));
        if frame.format != FORMAT_JSON {
            // basic_server converts XML to JSON (tinyxml2); ours serves JSON only
            return plain("Data type error,error code:57345");
        }
        let parsed = serde_json::from_slice::<Value>(&frame.payload).ok();
        let device = parsed.as_ref().map(|r| &r["PatrolDevice"]);
        let fields = device.and_then(|d| {
            Some((
                d.get("Type")?.as_i64()?,
                d.get("Command")?.as_i64()?,
                d.get("Items")?,
            ))
        });
        let Some((kind, command, items)) = fields else {
            return plain("JSON format parsing failed,error code:57346");
        };
        let items = match self.dispatch(kind, command, items, frame, from, clients) {
            Answer::Silent => return None,
            Answer::Items(v) => v,
            Answer::Code(c) => {
                let mut v = serde_json::json!({ "ErrorCode": c });
                if let Some(m) = error::message(c) {
                    v["ErrorMessage"] = Value::from(m);
                }
                v
            }
        };
        Some(Frame::answering(
            frame,
            frame.message_id,
            document(kind, command, &items),
        ))
    }

    fn dispatch(
        &self,
        kind: i64,
        command: i64,
        items: &Value,
        frame: &Frame,
        from: SocketAddr,
        clients: &Clients,
    ) -> Answer {
        let restricted = frame.from_type == 1;
        match (kind, command) {
            (100, 100) => {
                clients.heartbeat(from, frame);
                Answer::Code(error::OK)
            }
            (2, 21) => self.steer(items, false),
            (2, 25) => self.steer(items, true),
            (2, 22) => code(self.motion(items)),
            (2, 23) => code(self.gait(items)),
            (2, 24) => code(self.charge(items)),
            (1101, 4) => code(self.direction(items, restricted)),
            (1101, 5) => code(self.usage_mode(items, restricted)),
            (1101, 9) => code(self.align()),
            // GetInfo answers only its ErrorCode
            (1002, 2) => Answer::Code(error::OK),
            (1002, 3) => Answer::Items(
                serde_json::to_value(app_reports::error_items(&self.robot())).unwrap_or_default(),
            ),
            (3001, 0) if restricted => Answer::Code(error::NO_PERMISSION),
            (3001, 0) => Answer::Items(serde_json::json!({ "Sn": self.cfg.sn })),
            (2003, 1) => self.release_note(),
            key if UNSUPPORTED.contains(&key) => {
                info!(kind, command, "app command not served (yet)");
                Answer::Code(error::UNSUPPORTED_FUNCTION)
            }
            _ => {
                warn!(kind, command, "unknown app command");
                Answer::Code(error::UNSUPPORTED_TYPE)
            }
        }
    }

    fn robot(&self) -> app_reports::RobotView {
        self.links.snapshots.robot().unwrap_or_default()
    }

    /// cmdHandle_Steer 0x18a340 / cmdHandle_RealSteer 0x18a990: X, Y, Yaw to the operator; taken without a reply.
    fn steer(&self, items: &Value, metric: bool) -> Answer {
        if ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            .iter()
            .any(|k| items.get(k).is_none())
        {
            return Answer::Code(error::MISSING_FIELD);
        }
        if items["X"].is_string() {
            return Answer::Code(error::MISMATCHED_TYPE);
        }
        // X not a float (m20tool's "X":0 gotcha) is a zero steer
        let axis = |key: &str| items[key].as_f64().unwrap_or(0.0);
        let v = if items["X"].is_f64() {
            [axis("X"), axis("Y"), axis("Yaw")]
        } else {
            [0.0; 3]
        };
        if metric {
            self.links.operator.velocity(v, self.links.velocity_limits);
        } else {
            let reverse = self.links.usage.direction.load(Ordering::Relaxed) == 1;
            let k = if reverse { -1.0 } else { 1.0 };
            self.links.operator.steer([k * v[0], k * v[1], v[2]]);
        }
        Answer::Silent
    }

    /// cmdHandle_Motion (0x18aff0): only stand, soft e-stop, lie and RL control (since 1.2.40), else 0xe008.
    fn motion(&self, items: &Value) -> Result<(), i32> {
        let target = match integer(items, "MotionParam")? {
            1 => Target::Stand,
            2 => Target::JointDamping,
            4 => Target::LieDown,
            17 => Target::RlControl,
            _ => return Err(error::NOT_ALLOWED),
        };
        self.links.operator.motion(target);
        Ok(())
    }

    /// cmdHandle_Gait 0x18b430: Extre is 0xe007 (empty authority_code.toml); an unknown code is dropped, answered 0.
    fn gait(&self, items: &Value) -> Result<(), i32> {
        let gait = u32::try_from(integer(items, "GaitParam")?).unwrap_or(0);
        if extre_gait(gait) {
            return Err(error::NO_PERMISSION);
        }
        if known_gait(gait) {
            self.links.operator.gait(gait);
        } else {
            warn!(
                gait = format!("{gait:#x}"),
                "unknown gait from the app; dropped"
            );
        }
        Ok(())
    }

    /// cmdHandle_Charge 0x18b870: 1 dock, 0 undock into docking's request slot; any value is answered 0.
    fn charge(&self, items: &Value) -> Result<(), i32> {
        let request = match integer(items, "Charge")? {
            1 => Request::Dock,
            0 => Request::Undock,
            other => {
                warn!(
                    charge = other,
                    "unknown charge command from the app; ignored"
                );
                return Ok(());
            }
        };
        info!(?request, "app dock command");
        *self
            .links
            .dock
            .request
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(request);
        Ok(())
    }

    /// cmdHandle_Direction (0x18c040): the restricted client class 0xe007; any value stored (1 reverses the steer).
    fn direction(&self, items: &Value, restricted: bool) -> Result<(), i32> {
        if restricted {
            return Err(error::NO_PERMISSION);
        }
        let d = integer(items, "Direction")?;
        self.links
            .usage
            .direction
            .store(d as i32, Ordering::Relaxed);
        Ok(())
    }

    /// cmdHandle_ControlUsage (0x18c3c8): any mode stored, but 3 from the restricted client class (0xe007).
    fn usage_mode(&self, items: &Value, restricted: bool) -> Result<(), i32> {
        let mode = integer(items, "Mode")?;
        if restricted && mode == 3 {
            return Err(error::NO_PERMISSION);
        }
        *self
            .links
            .usage
            .request
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(mode as i32);
        Ok(())
    }

    /// cmdHandle_AligningBodyPosture 0x18e630: 0xe009 unless idle; crate::basic_server runs the recovery.
    fn align(&self) -> Result<(), i32> {
        if self.robot().state != Some(crate::state_machine::State::Passive) {
            return Err(error::FAILED);
        }
        self.links.usage.align.store(true, Ordering::Relaxed);
        Ok(())
    }

    /// cmdHandle_GetReleaseNote (0x191b10): the file's JSON as the Items, or 0xe00b with basic_server's message.
    fn release_note(&self) -> Answer {
        let failure = |message: &str| {
            Answer::Items(
                serde_json::json!({ "ErrorCode": error::INTERNAL, "ErrorMessage": message }),
            )
        };
        let Ok(text) = std::fs::read_to_string(&self.cfg.release_note_path) else {
            return failure("Can not open file");
        };
        match serde_json::from_str::<Value>(&text) {
            Ok(note) => Answer::Items(note),
            Err(_) => failure("Parse file fail"),
        }
    }

    /// UploadThread's reports this period (their payloads): motion always, device and basic when `slow`.
    pub fn reports(&self, slow: bool) -> Vec<Vec<u8>> {
        let robot = self.robot();
        let mut out = vec![document(1002, 4, &app_reports::motion_items(&robot))];
        if slow {
            let packs = self.links.snapshots.packs();
            out.push(document(
                1002,
                5,
                &app_reports::device_items(&robot, packs.as_ref()),
            ));
            out.push(document(
                1002,
                6,
                &app_reports::basic_items(&robot, &self.cfg.version),
            ));
        }
        out
    }

    /// ExceptionHandle's error report's payload.
    pub fn error_report(&self) -> Vec<u8> {
        document(1002, 3, &app_reports::error_items(&self.robot()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> (App, Links) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let mut cfg = crate::config::Config::load(&path).unwrap().app_server;
        cfg.sn = "M20TEST0001".into();
        let links = Links {
            operator: Arc::default(),
            usage: Arc::default(),
            dock: Arc::default(),
            snapshots: Arc::default(),
            velocity_limits: [2.0, 1.0, 2.0],
        };
        (App::new(cfg, links.clone()), links)
    }

    fn from() -> SocketAddr {
        "127.0.0.1:5000".parse().unwrap()
    }

    /// One frame's reply (header and PatrolDevice), none for a command answered with silence.
    fn reply_to(app: &App, frame: &Frame) -> Option<(Frame, Value)> {
        let reply = app.handle(frame, from(), &Clients::default())?;
        assert_eq!(
            reply.message_id, frame.message_id,
            "the message id is echoed"
        );
        let v: Value = serde_json::from_slice(&reply.payload).unwrap();
        let d = v["PatrolDevice"].clone();
        Some((reply, d))
    }

    fn frame(kind: i64, command: i64, items: Value) -> Frame {
        let root = serde_json::json!({
            "PatrolDevice": { "Type": kind, "Command": command, "Time": "2026-11-25 10:00:00", "Items": items }
        });
        Frame::json(42, serde_json::to_vec(&root).unwrap())
    }

    fn send(app: &App, kind: i64, command: i64, items: Value) -> Value {
        let (_, d) = reply_to(app, &frame(kind, command, items)).expect("a reply");
        assert_eq!(
            (d["Type"].as_i64(), d["Command"].as_i64()),
            (Some(kind), Some(command))
        );
        d
    }

    fn code(d: &Value) -> i32 {
        d["Items"]["ErrorCode"].as_i64().unwrap() as i32
    }

    #[test]
    fn a_reply_is_build_response_with_error_code_s_document() {
        let (app, _) = app();
        let (reply, _) =
            reply_to(&app, &frame(2, 22, serde_json::json!({"MotionParam": 9}))).unwrap();
        let text = String::from_utf8(reply.payload).unwrap();
        // Items first, then Type, Command, Time with milliseconds; the code's message from gs_CmdErrorMsg
        assert!(
            text.starts_with(
                r#"{"PatrolDevice":{"Items":{"ErrorCode":57352,"ErrorMessage":"Not allowed operation"},"Type":2,"Command":22,"Time":""#
            ),
            "{text}"
        );
        assert_eq!(
            text.len() - text.rfind("\"Time\":\"").unwrap(),
            7 + 1 + 23 + 3
        );
        // success has no ErrorMessage
        let ok = send(&app, 100, 100, serde_json::json!({}));
        assert_eq!(ok["Items"], serde_json::json!({"ErrorCode": 0}));
    }

    #[test]
    fn a_steer_is_the_operator_s_normalized_command_and_direction_reverses_it() {
        let (app, links) = app();
        let steer = serde_json::json!({"X": 0.5, "Y": -0.2, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 3.0});
        assert!(
            reply_to(&app, &frame(2, 21, steer.clone())).is_none(),
            "a steer taken is not answered"
        );
        let c = links.operator.take().unwrap();
        // clamped to 1 on every axis
        assert_eq!((c.velocity, c.steer), (Some([0.5, -0.2, 1.0]), true));
        let mut reverse = frame(1101, 4, serde_json::json!({"Direction": 1}));
        reverse.from_type = 0;
        assert!(reply_to(&app, &reverse).is_some());
        let _ = reply_to(&app, &frame(2, 21, steer));
        assert_eq!(
            links.operator.take().unwrap().velocity,
            Some([-0.5, 0.2, 1.0])
        );
        // an integer X (m20tool's "X":0 gotcha) is a zero steer; a string X or a missing key is refused
        let integer =
            serde_json::json!({"X": 1, "Y": 0.5, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.5});
        assert!(reply_to(&app, &frame(2, 21, integer)).is_none());
        assert_eq!(links.operator.take().unwrap().velocity, Some([0.0; 3]));
        let text = serde_json::json!({"X": "0.1", "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0});
        assert_eq!(code(&send(&app, 2, 21, text)), error::MISMATCHED_TYPE);
        let missing = serde_json::json!({"X": 0.1, "Y": 0.0, "Yaw": 0.0});
        assert_eq!(code(&send(&app, 2, 21, missing)), error::MISSING_FIELD);
        assert!(links.operator.take().is_none());
    }

    #[test]
    fn a_real_steer_is_a_twist_clamped_by_cmd_vel_s_limits_and_not_reversed() {
        let (app, links) = app();
        links.usage.direction.store(1, Ordering::Relaxed);
        let twist = serde_json::json!({"X": 3.0, "Y": 0.25, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.5});
        assert!(reply_to(&app, &frame(2, 25, twist)).is_none());
        let c = links.operator.take().unwrap();
        assert_eq!((c.velocity, c.steer), (Some([2.0, 0.25, 0.5]), false));
    }

    #[test]
    fn motion_params_are_the_controller_s_targets() {
        let (app, links) = app();
        for (param, target) in [
            (1, Target::Stand),
            (4, Target::LieDown),
            (2, Target::JointDamping),
            (17, Target::RlControl),
        ] {
            assert_eq!(
                code(&send(
                    &app,
                    2,
                    22,
                    serde_json::json!({"MotionParam": param})
                )),
                error::OK
            );
            assert_eq!(links.operator.take().unwrap().target, target);
        }
        assert_eq!(
            code(&send(&app, 2, 22, serde_json::json!({"MotionParam": 3}))),
            error::NOT_ALLOWED
        );
        assert_eq!(
            code(&send(&app, 2, 22, serde_json::json!({}))),
            error::MISSING_FIELD
        );
        // a string goes through stoi
        assert_eq!(
            code(&send(&app, 2, 22, serde_json::json!({"MotionParam": "4"}))),
            error::OK
        );
        assert_eq!(links.operator.take().unwrap().target, Target::LieDown);
    }

    #[test]
    fn gaits_are_switched_and_extre_ones_refused() {
        let (app, links) = app();
        assert_eq!(
            code(&send(&app, 2, 23, serde_json::json!({"GaitParam": 0x3002}))),
            error::OK
        );
        let c = links.operator.take().unwrap();
        assert_eq!((c.gait, c.gait_seq), (0x3002, 1));
        // SwitchGait logs and drops an unknown code; the reply is still 0
        assert_eq!(
            code(&send(&app, 2, 23, serde_json::json!({"GaitParam": 0x1234}))),
            error::OK
        );
        assert_eq!(
            code(&send(&app, 2, 23, serde_json::json!({"GaitParam": 0x2002}))),
            error::NO_PERMISSION
        );
        assert!(links.operator.take().is_none());
    }

    #[test]
    fn charge_usage_mode_direction_and_align_reach_their_slots() {
        let (app, links) = app();
        assert_eq!(
            code(&send(&app, 2, 24, serde_json::json!({"Charge": 1}))),
            error::OK
        );
        assert_eq!(*links.dock.request.lock().unwrap(), Some(Request::Dock));
        assert_eq!(
            code(&send(&app, 2, 24, serde_json::json!({"Charge": 0}))),
            error::OK
        );
        assert_eq!(*links.dock.request.lock().unwrap(), Some(Request::Undock));
        assert_eq!(
            code(&send(&app, 1101, 5, serde_json::json!({"Mode": 2}))),
            error::OK
        );
        assert_eq!(*links.usage.request.lock().unwrap(), Some(2));
        assert_eq!(
            code(&send(&app, 1101, 5, serde_json::json!({}))),
            error::MISSING_FIELD
        );
        // the restricted client class (header byte 1 = 0x91) may not set Mode 3 or the direction, nor read the SN
        assert_eq!(
            code(&send(&app, 1101, 5, serde_json::json!({"Mode": 3}))),
            error::NO_PERMISSION
        );
        assert_eq!(
            code(&send(&app, 1101, 4, serde_json::json!({"Direction": 1}))),
            error::NO_PERMISSION
        );
        assert_eq!(
            code(&send(&app, 3001, 0, serde_json::json!({}))),
            error::NO_PERMISSION
        );
        let mut open = frame(1101, 4, serde_json::json!({"Direction": 1}));
        open.from_type = 0;
        let (reply, d) = reply_to(&app, &open).unwrap();
        assert_eq!((code(&d), reply.from_type), (error::OK, 0));
        assert_eq!(links.usage.direction.load(Ordering::Relaxed), 1);
        // the fall recovery only from Idle
        assert_eq!(
            code(&send(&app, 1101, 9, serde_json::json!({}))),
            error::FAILED
        );
        links.snapshots.offer_robot(app_reports::RobotView {
            state: Some(crate::state_machine::State::Passive),
            ..Default::default()
        });
        assert_eq!(code(&send(&app, 1101, 9, serde_json::json!({}))), error::OK);
        assert!(links.usage.align.load(Ordering::Relaxed));
    }

    #[test]
    fn the_sn_and_release_note_come_from_the_config() {
        let (mut app, _) = app();
        let mut sn = frame(3001, 0, serde_json::json!({}));
        sn.from_type = 0;
        let (_, d) = reply_to(&app, &sn).unwrap();
        assert_eq!(d["Items"], serde_json::json!({"Sn": "M20TEST0001"}));
        let path =
            std::env::temp_dir().join(format!("m20_release_note_{}.json", std::process::id()));
        std::fs::write(&path, r#"{"version":"1.2.43","notes":["x"]}"#).unwrap();
        app.cfg.release_note_path = path.display().to_string();
        let d = send(&app, 2003, 1, serde_json::json!({}));
        assert_eq!(
            d["Items"],
            serde_json::json!({"version": "1.2.43", "notes": ["x"]})
        );
        std::fs::write(&path, "not json").unwrap();
        let d = send(&app, 2003, 1, serde_json::json!({}));
        std::fs::remove_file(&path).unwrap();
        assert_eq!(
            (code(&d), d["Items"]["ErrorMessage"].as_str()),
            (error::INTERNAL, Some("Parse file fail"))
        );
        let d = send(&app, 2003, 1, serde_json::json!({}));
        assert_eq!(d["Items"]["ErrorMessage"], "Can not open file");
    }

    #[test]
    fn the_undecided_commands_answer_unsupported_function_and_unknown_ones_unsupported_type() {
        let (app, links) = app();
        for (kind, command) in UNSUPPORTED {
            assert_eq!(
                code(&send(&app, kind, command, serde_json::json!({}))),
                error::UNSUPPORTED_FUNCTION
            );
        }
        assert_eq!(
            code(&send(&app, 77, 1, serde_json::json!({}))),
            error::UNSUPPORTED_TYPE
        );
        assert_eq!(
            send(&app, 1002, 2, serde_json::json!({}))["Items"],
            serde_json::json!({"ErrorCode": 0})
        );
        assert_eq!(
            send(&app, 1002, 3, serde_json::json!({}))["Items"],
            serde_json::json!({"ErrorList": []})
        );
        assert!(links.operator.take().is_none());
    }

    #[test]
    fn bad_frames_get_basic_server_s_plain_text_errors() {
        let (app, _) = app();
        let text = |f: Frame| {
            String::from_utf8(app.handle(&f, from(), &Clients::default()).unwrap().payload).unwrap()
        };
        let parse = "JSON format parsing failed,error code:57346";
        assert_eq!(text(Frame::json(1, "{not json")), parse);
        assert_eq!(
            text(Frame::json(1, r#"{"PatrolDevice":{"Items":{}}}"#)),
            parse
        );
        // Items is required, a heartbeat's too
        assert_eq!(
            text(Frame::json(
                1,
                r#"{"PatrolDevice":{"Type":100,"Command":100}}"#
            )),
            parse
        );
        let xml = Frame {
            format: crate::app_frame::FORMAT_XML,
            ..Frame::json(1, "<PatrolDevice/>")
        };
        assert_eq!(text(xml), "Data type error,error code:57345");
    }

    #[test]
    fn the_reports_are_the_motion_one_each_period_and_device_and_basic_on_the_slow_ones() {
        let (app, links) = app();
        let commands = |payloads: Vec<Vec<u8>>| -> Vec<i64> {
            payloads
                .iter()
                .map(|p| {
                    serde_json::from_slice::<Value>(p).unwrap()["PatrolDevice"]["Command"]
                        .as_i64()
                        .unwrap()
                })
                .collect()
        };
        assert_eq!(commands(app.reports(false)), [4]);
        links.snapshots.offer_robot(app_reports::RobotView {
            state: Some(crate::state_machine::State::RlControl),
            gait: 0x3002,
            ..Default::default()
        });
        let payloads = app.reports(true);
        assert_eq!(commands(payloads.clone()), [4, 5, 6]);
        let basic: Value = serde_json::from_slice(&payloads[2]).unwrap();
        let b = &basic["PatrolDevice"]["Items"]["BasicStatus"];
        assert_eq!(
            (b["MotionState"].as_i64(), b["Gait"].as_i64()),
            (Some(17), Some(0x3002))
        );
        // floats keep their decimal point (the app ignores an integer X)
        let text = String::from_utf8(payloads[0].clone()).unwrap();
        assert!(text.contains(r#""Roll":0.0"#), "{text}");
        let errors: Value = serde_json::from_slice(&app.error_report()).unwrap();
        assert_eq!(errors["PatrolDevice"]["Command"], 3);
    }
}
