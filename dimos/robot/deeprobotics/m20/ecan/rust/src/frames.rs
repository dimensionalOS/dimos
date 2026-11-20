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
use m20_battery::BatteryReport;
use m20_ecan::{decode_ethernet, Calibration, Direction, Message};

use crate::convert;

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
}

impl Published {
    /// The encoded message(s), as they go out on the outputs.
    pub fn encoded(&self) -> Vec<Vec<u8>> {
        match self {
            Published::JointState(m) | Published::JointCommand(m) => vec![m.encode()],
            Published::Battery(b) => vec![b.left.encode(), b.right.encode(), b.combined.encode()],
        }
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
            Ok(_) => Ok(None),
            Err(error) => Err(error.to_string()),
        }
    }
}
