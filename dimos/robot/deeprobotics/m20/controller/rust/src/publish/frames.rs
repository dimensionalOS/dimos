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

//! One motor-bus frame to the messages it yields, the same whoever owns the bus: frames rl_deploy exchanged (shim
//! tap) or frames this module's owner exchanged.

use lcm_msgs::sensor_msgs::{BatteryState, JointState};
use lcm_msgs::std_msgs::{Bool, Int32};
use m20_battery::BatteryReport;
use m20_ecan::{decode_ethernet, Calibration, Direction, Message};

use crate::publish::convert;

/// Both packs and the combined battery from one battery frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Batteries {
    pub left: BatteryState,
    pub right: BatteryState,
    pub combined: BatteryState,
}

/// A message for one output.
#[derive(Clone, Debug, PartialEq)]
pub enum Published {
    JointState(JointState),
    JointCommand(JointState),
    Battery(Box<Batteries>),
    /// /HES_STATUS: the 0x8a frame's hard-stop byte on every frame (rl_deploy handlEcanStatus 0x197c78)
    HardStop(Int32),
}

impl Published {
    /// The encoded message(s), as they go out on the outputs.
    pub fn encoded(&self) -> Vec<Vec<u8>> {
        match self {
            Published::JointState(m) | Published::JointCommand(m) => vec![m.encode()],
            Published::Battery(b) => vec![b.left.encode(), b.right.encode(), b.combined.encode()],
            Published::HardStop(m) => vec![m.encode()],
        }
    }
}

/// hardware_estop from a /HES_STATUS: true while the hard-stop byte is non-zero.
pub fn hardware_estop(hes_status: &Int32) -> Bool {
    Bool {
        data: hes_status.data != 0,
    }
}

pub struct FrameHandler {
    calibration: Calibration,
    frame_id: String,
}

impl FrameHandler {
    pub fn new(frame_id: &str) -> Self {
        FrameHandler {
            calibration: Calibration::m20(),
            frame_id: frame_id.to_string(),
        }
    }

    /// The message one frame yields, if any; Err for a frame that does not decode.
    pub fn handle(
        &self,
        timestamp_ns: u64,
        direction: Direction,
        frame: &[u8],
    ) -> Result<Option<Published>, String> {
        let frame_id = self.frame_id.as_str();
        match decode_ethernet(frame, direction) {
            Ok((_, Message::JointStates { joints, .. })) => Ok(Some(Published::JointState(
                convert::measured(&joints, &self.calibration, timestamp_ns, frame_id),
            ))),
            Ok((_, Message::JointCommands(commands))) => Ok(Some(Published::JointCommand(
                convert::commanded(&commands, &self.calibration, timestamp_ns, frame_id),
            ))),
            Ok((_, message @ Message::Battery(_))) => {
                let report = BatteryReport::from_message(&message).map_err(|e| e.to_string())?;
                Ok(Some(Published::Battery(Box::new(Batteries {
                    left: convert::pack_state(&report.left, "left", timestamp_ns, frame_id),
                    right: convert::pack_state(&report.right, "right", timestamp_ns, frame_id),
                    combined: convert::combined_state(
                        &[&report.left, &report.right],
                        timestamp_ns,
                        frame_id,
                    ),
                }))))
            }
            Ok((_, Message::EcanStatus(status))) => Ok(Some(Published::HardStop(Int32 {
                data: status.hard_stop as i32,
            }))),
            Ok(_) => Ok(None),
            Err(error) => Err(error.to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ecan_status_frame(hard_stop: u8) -> Vec<u8> {
        let mut payload = vec![0u8; m20_ecan::message::ECAN_STATUS_LEN];
        payload[0] = hard_stop;
        let body =
            m20_ecan::frame::encode(0, 6, 0x21, m20_ecan::FrameType::EcanStatus, &payload).unwrap();
        m20_ecan::ethernet::wrap([2, 0, 0, 0, 0, 1], &body)
    }

    #[test]
    fn each_ecan_status_frame_publishes_its_hard_stop_byte() {
        let handler = FrameHandler::new("base_link");
        for value in [0, 1, 0, 2] {
            let frame = ecan_status_frame(value);
            let published = handler.handle(0, Direction::FromBoard, &frame);
            assert_eq!(
                published,
                Ok(Some(Published::HardStop(Int32 { data: value as i32 })))
            );
            // hardware_estop: pressed on any non-zero byte
            let Ok(Some(Published::HardStop(hes))) = published else {
                unreachable!()
            };
            assert_eq!(hardware_estop(&hes).data, value != 0);
        }
    }
}
