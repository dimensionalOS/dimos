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

//! The RTP fixed header (RFC 3550), which is all the videohub stream needs.

pub struct Packet<'a> {
    pub sequence_number: u16,
    pub timestamp: u32,
    pub marker: bool,
    pub payload: &'a [u8],
}

/// None for anything that is not a version-2 RTP packet with a complete header.
pub fn parse(buf: &[u8]) -> Option<Packet<'_>> {
    if buf.len() < 12 || buf[0] >> 6 != 2 {
        return None;
    }
    let csrc_count = (buf[0] & 0x0F) as usize;
    let has_extension = buf[0] & 0x10 != 0;
    let padded = buf[0] & 0x20 != 0;
    let mut offset = 12 + 4 * csrc_count;
    if has_extension {
        let words = u16::from_be_bytes([*buf.get(offset + 2)?, *buf.get(offset + 3)?]) as usize;
        offset += 4 + 4 * words;
    }
    let end = buf.len() - if padded { *buf.last()? as usize } else { 0 };
    Some(Packet {
        sequence_number: u16::from_be_bytes([buf[2], buf[3]]),
        timestamp: u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]),
        marker: buf[1] & 0x80 != 0,
        payload: buf.get(offset..end)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_header_extension_and_padding() {
        // v2, padding, extension, 1 csrc; marker; seq 0x0102; ts 0x03040506; ssrc; csrc;
        // extension (id, 1 word); payload 0xAA 0xBB; 2 bytes of padding.
        let buf = [
            0xB1, 0xE0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0, 0, 0, 1, 0, 0, 0, 2, 0xBE, 0xDE,
            0x00, 0x01, 9, 9, 9, 9, 0xAA, 0xBB, 0x00, 0x02,
        ];
        let p = parse(&buf).expect("valid");
        assert_eq!(p.sequence_number, 0x0102);
        assert_eq!(p.timestamp, 0x03040506);
        assert!(p.marker);
        assert_eq!(p.payload, &[0xAA, 0xBB]);
        assert!(parse(&buf[..8]).is_none());
        assert!(parse(&[0u8; 12]).is_none());
    }
}
