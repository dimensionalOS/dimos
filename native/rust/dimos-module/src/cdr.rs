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

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Generated CDR codecs for native module ports.
use dimos_generated_messages::codec::Message;
use std::io;

pub fn encode<T: Message>(message: &T) -> io::Result<Vec<u8>> {
    message.encode()
}

pub fn decode<T: Message>(bytes: &[u8]) -> io::Result<T> {
    T::decode(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_messages::{geometry_msgs::msg::PoseStamped, std_msgs::msg::Int32};

    #[test]
    fn nested_fields_and_both_byte_orders() {
        let mut message = PoseStamped::default();
        message.header.frame_id = "map".into();
        message.header.stamp.sec = 1700000000;
        message.header.stamp.nanosec = 123456789;
        message.pose.position.x = 1.5;
        message.pose.orientation.w = 1.0;
        assert_eq!(
            decode::<PoseStamped>(&encode(&message).unwrap()).unwrap(),
            message
        );
        assert_eq!(
            decode::<PoseStamped>(&message.encode_endian(false).unwrap()).unwrap(),
            message
        );
    }

    #[test]
    fn malformed_payload_is_a_decode_error() {
        let mut bytes = encode(&Int32 { data: 1234 }).unwrap();
        assert_eq!(
            decode::<Int32>(&bytes[..bytes.len() - 1])
                .unwrap_err()
                .kind(),
            io::ErrorKind::InvalidData
        );
        bytes.push(0);
        assert_eq!(
            decode::<Int32>(&bytes).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        bytes[0] = 255;
        assert_eq!(
            decode::<Int32>(&bytes).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}
