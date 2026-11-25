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

//! basic_server's frames (CmdHandle::PackingHeaderWithData 0x17f7c0, ParseHeader 0x182ce8): a 16-byte header
//! [eb, 90 + client class, eb 90, payload length LE16, message id LE16, format, 7 zeros], then the JSON (or XML) text.

pub const HEADER_SIZE: usize = 16;
/// header byte 8 (& 3): the payload's format
pub const FORMAT_XML: u8 = 0;
pub const FORMAT_JSON: u8 = 1;
/// header byte 1 is 0x90 + the client class (FromType): 1 is the restricted class
pub const FROM_TYPE_BASE: u8 = 0x90;

/// One frame: the client class and format (echoed in the reply), the message id and the payload text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub from_type: u8,
    pub message_id: u16,
    pub format: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    pub fn json(message_id: u16, payload: impl Into<Vec<u8>>) -> Frame {
        Frame {
            from_type: 1,
            message_id,
            format: FORMAT_JSON,
            payload: payload.into(),
        }
    }

    /// The reply to (or a report for) a client that sent `like`: its class and format.
    pub fn answering(like: &Frame, message_id: u16, payload: impl Into<Vec<u8>>) -> Frame {
        Frame {
            from_type: like.from_type,
            message_id,
            format: like.format,
            payload: payload.into(),
        }
    }

    /// The bytes on the wire; a payload over 65535 bytes is cut (the length field is 16 bits).
    pub fn encode(&self) -> Vec<u8> {
        let len = self.payload.len().min(u16::MAX as usize);
        let mut out = Vec::with_capacity(HEADER_SIZE + len);
        out.extend_from_slice(&[0xeb, FROM_TYPE_BASE + (self.from_type & 1), 0xeb, 0x90]);
        out.extend_from_slice(&(len as u16).to_le_bytes());
        out.extend_from_slice(&self.message_id.to_le_bytes());
        out.push(self.format);
        out.extend_from_slice(&[0; 7]);
        out.extend_from_slice(&self.payload[..len]);
        out
    }
}

/// Whether `b` starts with a header's magic (either client class).
fn magic_at(b: &[u8]) -> bool {
    b.len() >= 4 && b[0] == 0xeb && (b[1] == 0x90 || b[1] == 0x91) && b[2] == 0xeb && b[3] == 0x90
}

/// The frame at the start of `b` and the bytes it takes, None while incomplete; Err(skip) for bytes that start none.
pub fn decode(b: &[u8]) -> Result<Option<(Frame, usize)>, usize> {
    if b.len() < 4 {
        return Ok(None);
    }
    if !magic_at(b) {
        // resynchronize on the next 0xeb
        return Err(b[1..]
            .iter()
            .position(|&x| x == 0xeb)
            .map_or(b.len(), |p| p + 1));
    }
    if b.len() < HEADER_SIZE {
        return Ok(None);
    }
    let len = u16::from_le_bytes([b[4], b[5]]) as usize;
    if b.len() < HEADER_SIZE + len {
        return Ok(None);
    }
    let frame = Frame {
        from_type: b[1] - FROM_TYPE_BASE,
        message_id: u16::from_le_bytes([b[6], b[7]]),
        format: b[8] & 3,
        payload: b[HEADER_SIZE..HEADER_SIZE + len].to_vec(),
    };
    Ok(Some((frame, HEADER_SIZE + len)))
}

/// UdpServer::recvData: a datagram is one frame exactly (16 + length == its size), anything else is dropped.
pub fn decode_datagram(b: &[u8]) -> Option<Frame> {
    match decode(b) {
        Ok(Some((frame, used))) if used == b.len() => Some(frame),
        _ => None,
    }
}

/// A TCP stream's frames: split and merged frames reassembled (basic_server 1.2.37 / 1.2.42); bytes that start no
/// frame are skipped up to the next 0xeb (basic_server drops its whole buffer; its 1460-byte buffer is not copied).
#[derive(Debug, Default)]
pub struct Stream {
    buf: Vec<u8>,
    /// bytes skipped that started no frame
    pub skipped: u64,
}

impl Stream {
    /// The bytes read; the frames they complete.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(bytes);
        let mut out = Vec::new();
        let mut at = 0;
        loop {
            match decode(&self.buf[at..]) {
                Ok(Some((frame, used))) => {
                    out.push(frame);
                    at += used;
                }
                Ok(None) => break,
                Err(skip) => {
                    self.skipped += skip as u64;
                    at += skip;
                }
            }
        }
        self.buf.drain(..at);
        out
    }
}

/// basic_server's "Time": local time as YYYY-MM-DD HH:MM:SS.mmm.
pub fn time_string(unix_s: i64, millis: u32) -> String {
    // SAFETY: localtime_r writes only the tm it is given
    let tm = unsafe {
        let t: libc::time_t = unix_s as libc::time_t;
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&t, &mut tm).is_null() {
            return String::from("1970-01-01 00:00:00.000");
        }
        tm
    };
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        millis.min(999)
    )
}

/// The local time now as basic_server writes it.
pub fn now_string() -> String {
    let d = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    time_string(d.as_secs() as i64, d.subsec_millis())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// m20tool's heartbeatFrame(7) with Time "2026-11-25 10:00:00" (backend/basic_server.ts commandFrame).
    const HEARTBEAT: &str =
        r#"{"PatrolDevice":{"Type":100,"Command":100,"Time":"2026-11-25 10:00:00","Items":{}}}"#;

    #[test]
    fn a_frame_round_trips_with_m20tool_s_header() {
        let f = Frame::json(7, HEARTBEAT);
        let bytes = f.encode();
        let len = HEARTBEAT.len() as u8;
        assert_eq!(
            &bytes[..HEADER_SIZE],
            &[0xeb, 0x91, 0xeb, 0x90, len, 0, 7, 0, 1, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(decode(&bytes), Ok(Some((f.clone(), bytes.len()))));
        // the other client class and XML: byte 1 and byte 8, echoed in a reply
        let xml = Frame {
            from_type: 0,
            format: FORMAT_XML,
            ..f
        };
        let wire = xml.encode();
        assert_eq!((wire[1], wire[8]), (0x90, 0));
        let reply = Frame::answering(&decode_datagram(&wire).unwrap(), 7, "x").encode();
        assert_eq!((reply[1], reply[8]), (0x90, 0));
    }

    #[test]
    fn a_stream_reassembles_split_and_merged_frames_and_skips_garbage() {
        let a = Frame::json(1, HEARTBEAT).encode();
        let b = Frame::json(0x1234, "{}").encode();
        let mut wire = vec![0x00, 0xeb, 0x13];
        wire.extend_from_slice(&a);
        wire.extend_from_slice(&b);
        let mut s = Stream::default();
        let mut got = Vec::new();
        for chunk in wire.chunks(5) {
            got.extend(s.push(chunk));
        }
        assert_eq!(
            got.iter().map(|f| f.message_id).collect::<Vec<_>>(),
            [1, 0x1234]
        );
        assert_eq!(s.skipped, 3);
        assert!(s.buf.is_empty());
    }

    #[test]
    fn a_datagram_is_one_whole_frame_or_dropped() {
        let a = Frame::json(3, HEARTBEAT).encode();
        assert!(decode_datagram(&a).is_some());
        assert!(decode_datagram(&a[..a.len() - 1]).is_none());
        let mut two = a.clone();
        two.extend_from_slice(&a);
        assert!(decode_datagram(&two).is_none());
        let mut bad = a;
        bad[1] = 0x92;
        assert!(decode_datagram(&bad).is_none());
    }

    #[test]
    fn the_time_is_basic_server_s_format() {
        let t = time_string(0, 7);
        assert_eq!(t.len(), 23);
        assert_eq!((&t[4..5], &t[13..14], &t[19..]), ("-", ":", ".007"));
    }
}
