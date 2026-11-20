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
//! The vendor DrDDS message layouts this module speaks, as CDR (`/usr/local/include/dridl/dr_msgs/msg/*.h` on AOS).

use serde::{Deserialize, Serialize};

/// `dr_msgs::msg::MetaType`: a sequence number and a wall-clock stamp.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct Header {
    pub frame_id: u64,
    pub sec: i32,
    pub nsec: u32,
}

impl Header {
    /// The vendor's header: the sequence number in `frame_id`, the wall clock in the stamp.
    pub fn now(sequence: u64) -> Self {
        let since = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        Self {
            frame_id: sequence,
            sec: since.as_secs() as i32,
            nsec: since.subsec_nanos(),
        }
    }
}

/// `dr_msgs::msg::Steer` (`drdds::msg::dds_::Steer_`): on REAL_STEER, m/s and rad/s in x, y and yaw.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct Steer {
    pub header: Header,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// `dr_msgs::msg::MotionInfo` (`drdds::msg::dds_::MotionInfo_`): rl_deploy's state, ~19 Hz.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionInfo {
    pub header: Header,
    pub vel_x: f32,
    pub vel_y: f32,
    pub vel_yaw: f32,
    pub height: f32,
    pub motion_state: i32,
    pub gait: u32,
    pub payload: f32,
    pub remain_mile: f32,
}

/// `dr_msgs::msg::StdMsgInt32` (`drdds::msg::dds_::StdMsgInt32_`): HES_STATUS, 0 when the e-stop chain is clear.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct StdMsgInt32 {
    pub value: i32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cdr<T: Serialize>(value: &T) -> Vec<u8> {
        rustdds::cdr_encoding::to_vec::<T, byteorder::LittleEndian>(value).unwrap()
    }

    #[test]
    fn layouts_match_the_vendor_sizes() {
        // MetaType 16 + six f32; MetaType 16 + four f32, i32, u32, two f32; one i32
        assert_eq!(cdr(&Steer::default()).len(), 40);
        assert_eq!(cdr(&MotionInfo::default()).len(), 48);
        assert_eq!(cdr(&StdMsgInt32::default()).len(), 4);
        let steer = Steer {
            x: 1.0,
            yaw: -0.5,
            ..Steer::default()
        };
        let bytes = cdr(&steer);
        assert_eq!(&bytes[16..20], &1.0f32.to_le_bytes());
        assert_eq!(&bytes[36..40], &(-0.5f32).to_le_bytes());
    }
}
