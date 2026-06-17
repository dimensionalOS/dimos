// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// DynamicCloud: a per-voxel point cloud + sparse timestamped event log.
//
// Mirrors `dimos/msgs/nav_msgs/DynamicCloud.py`. Wire format
// (little-endian, packed):
//
//     u64   timestamp_nanos             // overall message timestamp
//     f32   voxel_size                  // meters per voxel edge
//     u16   frame_id_len
//     bytes frame_id                    // utf-8, frame_id_len bytes
//     u32   num_points
//     i32[N*3]  voxels                  // (x, y, z) interleaved
//     u32[N]    quantity                // per-point quantity
//     u32   num_events
//     u32[M]    event_indices           // indices into voxels (0 ≤ idx < N)
//     u64[M]    event_timestamps        // nanoseconds
//
// `num_events` is independent of `num_points`; events can be empty,
// can reference the same point multiple times, and don't need to cover
// every point. The python test at `test_dynamic_cloud.py::test_known_bytes`
// pins the byte fixture this file's `tests::known_bytes_matches_python`
// also asserts against — drift on either side breaks both tests.

use std::convert::TryInto;
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub struct DynamicCloud {
    pub timestamp_nanos: u64,
    pub voxel_size: f32,
    pub frame_id: String,
    /// Voxel keys (signed integer coords in voxel-grid space).
    pub voxels: Vec<(i32, i32, i32)>,
    /// Per-point unsigned integer (e.g. voxel health/hit count).
    pub quantity: Vec<u32>,
    /// Sparse event log: indices into `voxels`.
    pub event_indices: Vec<u32>,
    /// Sparse event log: timestamp (nanoseconds) for each event.
    pub event_timestamps: Vec<u64>,
}

#[derive(Debug)]
#[allow(dead_code)] // decode path is exercised by tests, not the binary
pub enum DecodeError {
    Truncated { needed: usize, got: usize },
    InvalidUtf8(std::str::Utf8Error),
    PayloadSizeMismatch { expected: usize, got: usize },
    EventIndexOutOfRange { index: u32, num_points: u32 },
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Truncated { needed, got } => {
                write!(f, "DynamicCloud: truncated (needed {needed}, got {got})")
            }
            DecodeError::InvalidUtf8(e) => write!(f, "DynamicCloud: invalid utf-8: {e}"),
            DecodeError::PayloadSizeMismatch { expected, got } => write!(
                f,
                "DynamicCloud: payload size mismatch (expected {expected} tail bytes, got {got})"
            ),
            DecodeError::EventIndexOutOfRange { index, num_points } => write!(
                f,
                "DynamicCloud: event index {index} out of range for {num_points} points"
            ),
        }
    }
}

impl std::error::Error for DecodeError {}

#[derive(Debug)]
pub enum EncodeError {
    FrameIdTooLong(usize),
    EventLengthMismatch { indices: usize, timestamps: usize },
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EncodeError::FrameIdTooLong(n) => {
                write!(f, "DynamicCloud: frame_id too long ({n} > 65535 bytes)")
            }
            EncodeError::EventLengthMismatch { indices, timestamps } => write!(
                f,
                "DynamicCloud: event arrays length mismatch (indices={indices}, timestamps={timestamps})"
            ),
        }
    }
}

impl std::error::Error for EncodeError {}

const HEADER_SIZE: usize = 8 + 4 + 2;
const U32_SIZE: usize = 4;

impl DynamicCloud {
    #[allow(dead_code)] // public API, used by tests
    pub fn new(voxel_size: f32, frame_id: impl Into<String>) -> Self {
        Self {
            timestamp_nanos: 0,
            voxel_size,
            frame_id: frame_id.into(),
            voxels: Vec::new(),
            quantity: Vec::new(),
            event_indices: Vec::new(),
            event_timestamps: Vec::new(),
        }
    }

    #[allow(dead_code)] // public API, used by tests
    pub fn len(&self) -> usize {
        self.voxels.len()
    }

    pub fn encode(&self) -> Result<Vec<u8>, EncodeError> {
        let frame_bytes = self.frame_id.as_bytes();
        if frame_bytes.len() > u16::MAX as usize {
            return Err(EncodeError::FrameIdTooLong(frame_bytes.len()));
        }
        if self.event_indices.len() != self.event_timestamps.len() {
            return Err(EncodeError::EventLengthMismatch {
                indices: self.event_indices.len(),
                timestamps: self.event_timestamps.len(),
            });
        }

        let num_points = self.voxels.len().min(self.quantity.len());
        let num_events = self.event_indices.len();

        let voxels_bytes = num_points * 3 * 4;
        let quantity_bytes = num_points * 4;
        let events_idx_bytes = num_events * 4;
        let events_ts_bytes = num_events * 8;
        let total = HEADER_SIZE
            + frame_bytes.len()
            + U32_SIZE
            + voxels_bytes
            + quantity_bytes
            + U32_SIZE
            + events_idx_bytes
            + events_ts_bytes;

        let mut buf = Vec::with_capacity(total);
        buf.extend_from_slice(&self.timestamp_nanos.to_le_bytes());
        buf.extend_from_slice(&self.voxel_size.to_le_bytes());
        buf.extend_from_slice(&(frame_bytes.len() as u16).to_le_bytes());
        buf.extend_from_slice(frame_bytes);
        buf.extend_from_slice(&(num_points as u32).to_le_bytes());

        for &(x, y, z) in &self.voxels[..num_points] {
            buf.extend_from_slice(&x.to_le_bytes());
            buf.extend_from_slice(&y.to_le_bytes());
            buf.extend_from_slice(&z.to_le_bytes());
        }
        for &q in &self.quantity[..num_points] {
            buf.extend_from_slice(&q.to_le_bytes());
        }

        buf.extend_from_slice(&(num_events as u32).to_le_bytes());
        for &idx in &self.event_indices {
            buf.extend_from_slice(&idx.to_le_bytes());
        }
        for &t in &self.event_timestamps {
            buf.extend_from_slice(&t.to_le_bytes());
        }

        Ok(buf)
    }

    #[allow(dead_code)] // decode path is exercised by tests, not the binary
    pub fn decode(data: &[u8]) -> Result<Self, DecodeError> {
        if data.len() < HEADER_SIZE {
            return Err(DecodeError::Truncated {
                needed: HEADER_SIZE,
                got: data.len(),
            });
        }

        let timestamp_nanos = u64::from_le_bytes(data[0..8].try_into().unwrap());
        let voxel_size = f32::from_le_bytes(data[8..12].try_into().unwrap());
        let frame_id_len = u16::from_le_bytes(data[12..14].try_into().unwrap()) as usize;
        let mut offset = HEADER_SIZE;

        let needed = offset + frame_id_len + U32_SIZE;
        if data.len() < needed {
            return Err(DecodeError::Truncated {
                needed,
                got: data.len(),
            });
        }

        let frame_id = std::str::from_utf8(&data[offset..offset + frame_id_len])
            .map_err(DecodeError::InvalidUtf8)?
            .to_string();
        offset += frame_id_len;

        let num_points =
            u32::from_le_bytes(data[offset..offset + U32_SIZE].try_into().unwrap()) as usize;
        offset += U32_SIZE;

        let voxels_bytes = num_points * 3 * 4;
        let quantity_bytes = num_points * 4;
        let needed_after_points = offset + voxels_bytes + quantity_bytes + U32_SIZE;
        if data.len() < needed_after_points {
            return Err(DecodeError::Truncated {
                needed: needed_after_points,
                got: data.len(),
            });
        }

        let mut voxels = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let base = offset + i * 12;
            let x = i32::from_le_bytes(data[base..base + 4].try_into().unwrap());
            let y = i32::from_le_bytes(data[base + 4..base + 8].try_into().unwrap());
            let z = i32::from_le_bytes(data[base + 8..base + 12].try_into().unwrap());
            voxels.push((x, y, z));
        }
        offset += voxels_bytes;

        let mut quantity = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let base = offset + i * 4;
            quantity.push(u32::from_le_bytes(data[base..base + 4].try_into().unwrap()));
        }
        offset += quantity_bytes;

        let num_events =
            u32::from_le_bytes(data[offset..offset + U32_SIZE].try_into().unwrap()) as usize;
        offset += U32_SIZE;

        let events_idx_bytes = num_events * 4;
        let events_ts_bytes = num_events * 8;
        let expected_tail = events_idx_bytes + events_ts_bytes;
        if data.len() - offset != expected_tail {
            return Err(DecodeError::PayloadSizeMismatch {
                expected: expected_tail,
                got: data.len() - offset,
            });
        }

        let mut event_indices = Vec::with_capacity(num_events);
        for i in 0..num_events {
            let base = offset + i * 4;
            let idx = u32::from_le_bytes(data[base..base + 4].try_into().unwrap());
            if num_points == 0 || idx as usize >= num_points {
                return Err(DecodeError::EventIndexOutOfRange {
                    index: idx,
                    num_points: num_points as u32,
                });
            }
            event_indices.push(idx);
        }
        offset += events_idx_bytes;

        let mut event_timestamps = Vec::with_capacity(num_events);
        for i in 0..num_events {
            let base = offset + i * 8;
            event_timestamps.push(u64::from_le_bytes(data[base..base + 8].try_into().unwrap()));
        }

        Ok(Self {
            timestamp_nanos,
            voxel_size,
            frame_id,
            voxels,
            quantity,
            event_indices,
            event_timestamps,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> DynamicCloud {
        DynamicCloud {
            timestamp_nanos: 1_500_000_000,
            voxel_size: 0.25,
            frame_id: "map".to_string(),
            voxels: vec![(1, -2, 3), (4, 5, -6)],
            quantity: vec![7, 8],
            event_indices: vec![0, 1, 0],
            event_timestamps: vec![1_000_000_000, 2_000_000_000, 1_500_000_000],
        }
    }

    // Same hex string as the Python KNOWN_BYTES fixture in
    // test_dynamic_cloud.py — keep them in sync.
    const KNOWN_BYTES_HEX: &str = concat!(
        "002f685900000000", // ts_ns = 1_500_000_000 LE
        "0000803e",         // voxel_size = 0.25 f32 LE
        "0300",             // frame_id_len = 3
        "6d6170",           // "map"
        "02000000",         // num_points = 2
        "01000000feffffff03000000",
        "0400000005000000faffffff",
        "0700000008000000",
        "03000000", // num_events = 3
        "000000000100000000000000",
        "00ca9a3b00000000",
        "0094357700000000",
        "002f685900000000",
    );

    fn hex_to_bytes(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn roundtrip() {
        let cloud = fixture();
        let bytes = cloud.encode().expect("encode");
        let decoded = DynamicCloud::decode(&bytes).expect("decode");
        assert_eq!(cloud, decoded);
    }

    #[test]
    fn known_bytes_matches_python() {
        let cloud = fixture();
        let bytes = cloud.encode().expect("encode");
        let expected = hex_to_bytes(KNOWN_BYTES_HEX);
        assert_eq!(bytes, expected, "encoded bytes drift from python fixture");
    }

    #[test]
    fn decode_known_bytes() {
        let bytes = hex_to_bytes(KNOWN_BYTES_HEX);
        let decoded = DynamicCloud::decode(&bytes).expect("decode");
        assert_eq!(decoded, fixture());
    }

    #[test]
    fn empty_cloud_roundtrip() {
        let cloud = DynamicCloud::new(0.1, "world");
        let bytes = cloud.encode().expect("encode");
        let decoded = DynamicCloud::decode(&bytes).expect("decode");
        assert_eq!(cloud, decoded);
        assert_eq!(decoded.len(), 0);
        assert!(decoded.event_indices.is_empty());
    }

    #[test]
    fn truncated_returns_err() {
        assert!(matches!(
            DynamicCloud::decode(&[0u8; 4]),
            Err(DecodeError::Truncated { .. })
        ));
    }

    #[test]
    fn payload_size_mismatch_returns_err() {
        let mut bytes = fixture().encode().unwrap();
        bytes.pop(); // chop a byte off the tail
        assert!(matches!(
            DynamicCloud::decode(&bytes),
            Err(DecodeError::PayloadSizeMismatch { .. })
        ));
    }

    #[test]
    fn event_index_out_of_range_returns_err() {
        let cloud = DynamicCloud {
            timestamp_nanos: 0,
            voxel_size: 0.1,
            frame_id: "x".to_string(),
            voxels: vec![(0, 0, 0)],
            quantity: vec![1],
            event_indices: vec![5],
            event_timestamps: vec![123],
        };
        let bytes = cloud.encode().unwrap();
        assert!(matches!(
            DynamicCloud::decode(&bytes),
            Err(DecodeError::EventIndexOutOfRange {
                index: 5,
                num_points: 1
            })
        ));
    }
}
