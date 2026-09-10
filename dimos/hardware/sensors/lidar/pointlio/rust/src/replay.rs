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

// L0 replay file reader/writer. Format: see REPLAY.md.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

pub use dimos_livox::pipeline::RawPoint;

const MAGIC: &[u8; 4] = b"PLIO";
const VERSION: u32 = 1;
const POINT_LEN: usize = 21;

/// One replay record, as the Mid360 module would publish it.
#[derive(Debug, Clone, PartialEq)]
pub enum Record {
    Imu {
        ts_ns: u64,
        /// rad/s
        gyro: [f64; 3],
        /// m/s^2
        acc: [f64; 3],
    },
    Lidar {
        start_ns: u64,
        points: Vec<RawPoint>,
    },
}

impl Record {
    /// When a live consumer would see it: the IMU stamp, or the frame end.
    pub fn avail_ns(&self) -> u64 {
        match self {
            Record::Imu { ts_ns, .. } => *ts_ns,
            Record::Lidar { start_ns, points } => {
                let last = points.iter().map(|p| p.offset_ns).max().unwrap_or(0);
                start_ns + u64::from(last)
            }
        }
    }

    fn kind(&self) -> u8 {
        match self {
            Record::Imu { .. } => 0,
            Record::Lidar { .. } => 1,
        }
    }
}

/// Write records in feed order: by availability time, IMU first on ties.
pub fn write(path: impl AsRef<Path>, hz: f64, records: &[Record]) -> io::Result<()> {
    let mut order: Vec<&Record> = records.iter().collect();
    order.sort_by_cached_key(|r| (r.avail_ns(), r.kind()));
    let mut w = BufWriter::new(File::create(path)?);
    w.write_all(MAGIC)?;
    w.write_all(&VERSION.to_le_bytes())?;
    w.write_all(&hz.to_le_bytes())?;
    for record in order {
        write_record(&mut w, record)?;
    }
    w.flush()
}

fn write_record(w: &mut impl Write, record: &Record) -> io::Result<()> {
    w.write_all(&[record.kind()])?;
    match record {
        Record::Imu { ts_ns, gyro, acc } => {
            w.write_all(&ts_ns.to_le_bytes())?;
            for v in gyro.iter().chain(acc) {
                w.write_all(&v.to_le_bytes())?;
            }
        }
        Record::Lidar { start_ns, points } => {
            w.write_all(&start_ns.to_le_bytes())?;
            w.write_all(&(points.len() as u32).to_le_bytes())?;
            let mut buf = Vec::with_capacity(points.len() * POINT_LEN);
            for p in points {
                for v in p.xyz_m {
                    buf.extend_from_slice(&v.to_le_bytes());
                }
                buf.extend_from_slice(&p.intensity.to_le_bytes());
                buf.extend_from_slice(&p.offset_ns.to_le_bytes());
                buf.push(p.tag);
            }
            w.write_all(&buf)?;
        }
    }
    Ok(())
}

/// Streams records out of a replay file in file order.
pub struct Reader {
    r: BufReader<File>,
    pub hz: f64,
}

impl Reader {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let mut r = BufReader::new(File::open(path)?);
        if &read_array::<4>(&mut r)? != MAGIC {
            return Err(invalid("not a PLIO file"));
        }
        let version = u32::from_le_bytes(read_array(&mut r)?);
        if version != VERSION {
            return Err(invalid(&format!("PLIO version {version}, want {VERSION}")));
        }
        let hz = f64::from_le_bytes(read_array(&mut r)?);
        Ok(Reader { r, hz })
    }

    fn read_record(&mut self) -> io::Result<Option<Record>> {
        let mut kind = [0u8; 1];
        if self.r.read(&mut kind)? == 0 {
            return Ok(None);
        }
        let r = &mut self.r;
        let record = match kind[0] {
            0 => {
                let ts_ns = u64::from_le_bytes(read_array(r)?);
                let mut v = [0f64; 6];
                for x in &mut v {
                    *x = f64::from_le_bytes(read_array(r)?);
                }
                Record::Imu {
                    ts_ns,
                    gyro: [v[0], v[1], v[2]],
                    acc: [v[3], v[4], v[5]],
                }
            }
            1 => {
                let start_ns = u64::from_le_bytes(read_array(r)?);
                let n = u32::from_le_bytes(read_array(r)?) as usize;
                let mut buf = vec![0u8; n * POINT_LEN];
                r.read_exact(&mut buf)?;
                let points = buf.chunks_exact(POINT_LEN).map(decode_point).collect();
                Record::Lidar { start_ns, points }
            }
            k => return Err(invalid(&format!("bad record kind {k}"))),
        };
        Ok(Some(record))
    }
}

impl Iterator for Reader {
    type Item = io::Result<Record>;

    fn next(&mut self) -> Option<Self::Item> {
        self.read_record().transpose()
    }
}

fn decode_point(b: &[u8]) -> RawPoint {
    let f = |i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
    RawPoint {
        xyz_m: [f(0), f(4), f(8)],
        intensity: f(12),
        offset_ns: u32::from_le_bytes(b[16..20].try_into().unwrap()),
        tag: b[20],
    }
}

fn read_array<const N: usize>(r: &mut impl Read) -> io::Result<[u8; N]> {
    let mut buf = [0u8; N];
    r.read_exact(&mut buf)?;
    Ok(buf)
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_in_feed_order() {
        let point = |offset_ns| RawPoint {
            xyz_m: [1.5, -2.0, 0.25],
            intensity: 0.5,
            offset_ns,
            tag: 3,
        };
        let imu = |ts_ns| Record::Imu {
            ts_ns,
            gyro: [0.1, -0.2, 0.3],
            acc: [0.0, 0.0, 9.80665],
        };
        // Frame ends at 150; the IMU at 150 ties and must come first.
        let frame = Record::Lidar {
            start_ns: 100,
            points: vec![point(0), point(50), point(20)],
        };
        let records = vec![imu(200), frame.clone(), imu(150), imu(100)];

        let dir = std::env::temp_dir().join(format!("plio-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("rt.plio");
        write(&path, 10.0, &records).unwrap();
        let reader = Reader::open(&path).unwrap();
        assert_eq!(reader.hz, 10.0);
        let got: Vec<Record> = reader.map(Result::unwrap).collect();
        std::fs::remove_dir_all(&dir).unwrap();

        assert_eq!(got, vec![imu(100), imu(150), frame, imu(200)]);
    }
}
