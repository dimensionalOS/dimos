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

//! The vendor app server on loopback with a fake app: m20tool's frames over UDP and TCP, the replies, the pushed
//! reports, the heartbeat timeout, and the commands reaching the operator's slot.

mod common;

use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::sync::Arc;
use std::time::{Duration, Instant};

use m20_controller::app_commands::{error, Links};
use m20_controller::app_frame::{decode_datagram, Frame, Stream};
use m20_controller::app_server::AppServer;
use m20_controller::config::AppServerConfig;
use m20_controller::operator::Operator;
use m20_controller::types::Target;
use serde_json::Value;

/// The TOML's server on loopback, on ports the OS picks, with a short heartbeat timeout.
fn server() -> (AppServer, Arc<Operator>, AppServerConfig) {
    let mut cfg = common::config().app_server;
    cfg.bind = "127.0.0.1".into();
    cfg.udp_port = 0;
    cfg.tcp_port = 0;
    cfg.heartbeat_timeout_s = 0.5;
    let operator = Arc::new(Operator::default());
    let links = Links {
        operator: operator.clone(),
        usage: Arc::default(),
        dock: Arc::default(),
        snapshots: Arc::default(),
        velocity_limits: [2.0, 1.0, 2.0],
    };
    (AppServer::start(&cfg, links).unwrap(), operator, cfg)
}

/// m20tool's commandFrame: the JSON with the float fields written with a decimal point.
fn command(kind: i64, command: i64, items: &str, id: u16) -> Vec<u8> {
    let json = format!(
        r#"{{"PatrolDevice":{{"Type":{kind},"Command":{command},"Time":"2026-11-25 10:00:00","Items":{items}}}}}"#
    );
    Frame::json(id, json).encode()
}

fn device(frame: &Frame) -> Value {
    serde_json::from_slice::<Value>(&frame.payload).unwrap()["PatrolDevice"].clone()
}

/// UDP frames until `want` matches one or the time runs out.
fn udp_until(sock: &UdpSocket, wait: Duration, want: impl Fn(&Frame) -> bool) -> Option<Frame> {
    let end = Instant::now() + wait;
    let mut buf = [0u8; 65536];
    while Instant::now() < end {
        if let Ok(n) = sock.recv(&mut buf) {
            if let Some(f) = decode_datagram(&buf[..n]).filter(&want) {
                return Some(f);
            }
        }
    }
    None
}

#[test]
fn a_udp_client_s_heartbeat_gets_reports_until_it_stops() {
    let (srv, operator, cfg) = server();
    let app = UdpSocket::bind("127.0.0.1:0").unwrap();
    app.set_read_timeout(Some(Duration::from_millis(20)))
        .unwrap();
    app.connect(srv.udp_addr).unwrap();
    app.send(&command(100, 100, "{}", 9)).unwrap();
    let reply = udp_until(&app, Duration::from_secs(1), |f| device(f)["Type"] == 100).unwrap();
    let d = device(&reply);
    assert_eq!(
        (d["Type"].as_i64(), d["Command"].as_i64()),
        (Some(100), Some(100))
    );
    assert_eq!(d["Items"]["ErrorCode"], 0);
    // the 10 Hz motion report, then the 2 Hz basic one
    assert_eq!(reply.message_id, 9);
    let motion = udp_until(&app, Duration::from_secs(1), |f| device(f)["Command"] == 4).unwrap();
    // pushed reports carry message id 0 and the heartbeat's header bytes
    assert_eq!(
        (motion.message_id, motion.from_type, motion.format),
        (0, 1, 1)
    );
    assert!(device(&motion)["Items"]["MotionStatus"].is_object());
    let basic = udp_until(&app, Duration::from_secs(1), |f| device(f)["Command"] == 6).unwrap();
    assert!(device(&basic)["Items"]["BasicStatus"].is_object());
    // a Steer reaches the operator (normalized, as the app's joystick)
    app.send(&command(100, 100, "{}", 11)).unwrap();
    app.send(&command(
        2,
        21,
        r#"{"X":0.5,"Y":0.0,"Z":0.0,"Roll":0.0,"Pitch":0.0,"Yaw":-0.25}"#,
        10,
    ))
    .unwrap();
    // taken without a reply
    let end = Instant::now() + Duration::from_secs(1);
    let c = loop {
        if let Some(c) = operator.take() {
            break c;
        }
        assert!(Instant::now() < end, "the steer never reached the operator");
        std::thread::sleep(Duration::from_millis(5));
    };
    assert_eq!((c.velocity, c.steer), (Some([0.5, 0.0, -0.25]), true));
    // silent past the heartbeat timeout: dropped, no more reports
    std::thread::sleep(Duration::from_secs_f64(cfg.heartbeat_timeout_s * 3.0));
    assert!(srv.clients.live().is_empty());
    while app.recv(&mut [0u8; 65536]).is_ok() {}
    assert!(udp_until(&app, Duration::from_millis(300), |_| true).is_none());
    srv.stop();
}

#[test]
fn a_tcp_client_s_split_and_merged_frames_are_answered_on_its_stream() {
    let (srv, operator, _) = server();
    let mut app = TcpStream::connect(srv.tcp_addr).unwrap();
    app.set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();
    let mut wire = command(100, 100, "{}", 1);
    wire.extend(command(2, 22, r#"{"MotionParam":1}"#, 2));
    wire.extend(command(2, 23, r#"{"GaitParam":8194}"#, 3));
    let (a, b) = wire.split_at(20);
    app.write_all(a).unwrap();
    std::thread::sleep(Duration::from_millis(30));
    app.write_all(b).unwrap();
    let mut stream = Stream::default();
    let mut replies = Vec::new();
    let end = Instant::now() + Duration::from_secs(2);
    let mut buf = [0u8; 65536];
    while replies.len() < 3 && Instant::now() < end {
        if let Ok(n) = app.read(&mut buf) {
            let frames = stream.push(&buf[..n]);
            replies.extend(
                frames
                    .into_iter()
                    .filter(|f| (1..=3).contains(&f.message_id)),
            );
        }
    }
    let codes: Vec<_> = replies
        .iter()
        .map(|f| {
            (
                f.message_id,
                device(f)["Items"]["ErrorCode"].as_i64().unwrap(),
            )
        })
        .collect();
    // the Extre jump (0x2002) needs the authorization code basic_server matches; the M20's is empty
    assert_eq!(codes, [(1, 0), (2, 0), (3, error::NO_PERMISSION as i64)]);
    assert_eq!(operator.take().unwrap().target, Target::Stand);
    assert_eq!(srv.clients.live().len(), 1);
    srv.stop();
}
