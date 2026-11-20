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

//! The ECAN backend against frames rl_deploy exchanged with the board on 2026-11-20 (moving session cut): the default
//! mapping reads the measured pose like m20-ecan's calibration, re-encoding rl_deploy's commands reproduces them, and
//! the charge rule plus encoder reproduce rl_deploy's charge-enable frames.

use m20_controller::config::Ecan;
use m20_controller::hardware::ecan::{
    decode_state, encode_charge_enable, encode_command, Charge, Mapping,
};
use m20_controller::types::JointCommand;
use m20_ecan::{decode_ethernet, tap, Calibration, Direction, Message};

const CAPTURE: &[u8] = include_bytes!("fixtures/moving_20261120_141603.pcapng");

#[test]
fn states_read_like_the_vendor_calibration() {
    let cfg = Ecan::default();
    let map = Mapping::new(&cfg).unwrap();
    let calibration = Calibration::m20();
    let mut n = 0;
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        let Ok((_, Message::JointStates { joints, .. })) = decode_ethernet(frame, direction) else {
            continue;
        };
        let ours = decode_state(frame, cfg.ethertype, &map).unwrap().unwrap();
        for j in &joints {
            let i = j.joint.index();
            let want = calibration.urdf_position(j.joint, j.position) as f64;
            assert!(
                (ours.q[i] - want).abs() < 1e-4,
                "{:?} {} vs {want}",
                j.joint,
                ours.q[i]
            );
            assert!((ours.dq[i] - calibration.urdf_rate(j.joint, j.velocity) as f64).abs() < 1e-5);
        }
        n += 1;
    }
    assert!(n >= 30);
}

#[test]
fn commands_re_encode_to_rl_deploys_bytes() {
    let cfg = Ecan::default();
    let map = Mapping::new(&cfg).unwrap();
    let calibration = Calibration::m20();
    let mut n = 0;
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        let Ok((header, Message::JointCommands(commands))) = decode_ethernet(frame, direction)
        else {
            continue;
        };
        // the controller sends one control word per frame; skip rl_deploy's one-cycle enable frames
        if commands[0].control_word != cfg.control_word {
            continue;
        }
        let mut cmd = JointCommand::default();
        for c in &commands {
            let i = c.joint.index();
            cmd.q[i] = calibration.urdf_position(c.joint, c.position) as f64;
            cmd.dq[i] = calibration.urdf_rate(c.joint, c.velocity) as f64;
            cmd.tau[i] = calibration.urdf_rate(c.joint, c.torque) as f64;
            cmd.kp[i] = c.kp as f64;
            cmd.kd[i] = c.kd as f64;
        }
        let ours = encode_command(
            &cmd,
            header.seq,
            frame[6..12].try_into().unwrap(),
            &cfg,
            &map,
        );
        // positions go through f64 and back; compare everything else exactly and positions to the f32 ulp
        let theirs = &frame[..ours.len()];
        for k in 0..16 {
            let (a, b) = (
                &ours[24 + k * 24..48 + k * 24],
                &theirs[24 + k * 24..48 + k * 24],
            );
            assert_eq!(a[..4], b[..4]);
            assert_eq!(a[8..], b[8..], "unit {k} gains / words");
            let (pa, pb) = (
                f32::from_le_bytes(a[4..8].try_into().unwrap()),
                f32::from_le_bytes(b[4..8].try_into().unwrap()),
            );
            assert!(
                (pa - pb).abs() <= 4.0 * f32::EPSILON * pb.abs().max(1.0),
                "unit {k}: {pa} vs {pb}"
            );
        }
        n += 1;
    }
    assert!(n >= 20);
}

#[test]
fn charge_enable_frames_match_rl_deploy() {
    let mut sent = 0;
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        if direction == Direction::ToBoard && frame[19] == 0xb3 {
            let mac: [u8; 6] = frame[6..12].try_into().unwrap();
            assert_eq!(
                encode_charge_enable(frame[16], mac, &Ecan::default()),
                frame
            );
            sent += 1;
        }
    }
    assert_eq!(sent, 12);
}

#[test]
fn charge_rule_on_the_bus() {
    // lying down (all leg gains zero after a standing command) then a battery frame off the dock: one probe of 3 frames
    let frames = tap::read_capture(CAPTURE).unwrap();
    let off_dock = frames
        .iter()
        .find(|(t, d, f)| {
            *d == Direction::FromBoard && f[19] == 0x87 && *t > 1_795_155_410_000_000_000
        })
        .unwrap()
        .2;
    let mut charge = Charge::new();
    let standing = JointCommand {
        kp: [60.0; 16],
        ..Default::default()
    };
    charge.on_command(&standing);
    charge.on_frame(off_dock);
    assert!(!charge.take_tick(), "never while standing");
    charge.on_command(&JointCommand::default());
    charge.on_frame(off_dock);
    assert!(charge.take_tick() && charge.take_tick() && charge.take_tick() && !charge.take_tick());
    charge.on_frame(off_dock);
    assert!(!charge.take_tick(), "one probe per lie-down");
}
