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

//! Tap mode and owner mode give the same messages for the same frames: the moving-session cut (joint states,
//! commands, battery) once as the shim's stream delivers it and once as the owner's Bus hands it over.

use m20_controller::publish::frames::FrameHandler;
use m20_ecan::tap::{read_capture, TapRecord};

const CAPTURE: &[u8] = include_bytes!("fixtures/moving_20261120_141603.pcapng");

#[test]
fn identical_outputs_across_modes() {
    let handler = FrameHandler::new("base_link");
    let frames = read_capture(CAPTURE).unwrap();
    // tap mode: each frame through the shim's datagram encoding and back
    let mut datagram = Vec::new();
    for (t, direction, frame) in &frames {
        TapRecord {
            timestamp_ns: *t,
            direction: *direction,
            ring_dropped: 0,
            send_dropped: 0,
            frame: frame.to_vec(),
        }
        .encode(&mut datagram);
    }
    let tap: Vec<_> = TapRecord::decode_datagram(&datagram)
        .unwrap()
        .into_iter()
        .map(|r| {
            handler
                .handle(r.timestamp_ns, r.direction, &r.frame)
                .unwrap()
        })
        .collect();
    // owner mode: the same frames as Bus::tick / Bus::drain return them
    let owner: Vec<_> = frames
        .iter()
        .map(|(t, d, f)| handler.handle(*t, *d, f).unwrap())
        .collect();
    let bytes =
        |v: &Vec<Option<m20_controller::publish::frames::Published>>| -> Vec<Option<Vec<Vec<u8>>>> {
            v.iter().map(|p| p.as_ref().map(|p| p.encoded())).collect()
        };
    assert_eq!(bytes(&tap), bytes(&owner));
    let count = |pick: fn(&m20_controller::publish::frames::Published) -> bool| {
        owner.iter().flatten().filter(|p| pick(p)).count()
    };
    use m20_controller::publish::frames::Published as P;
    assert!(count(|p| matches!(p, P::JointState(_))) >= 30);
    assert!(count(|p| matches!(p, P::JointCommand(_))) >= 30);
    assert_eq!(count(|p| matches!(p, P::Battery(_))), 3);
}
