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

//! Timestamp projection through a runtime schema; re_cdr owns wire alignment.
//! Entire payloads stay opaque and unchanged. No message classes are duplicated.

use anyhow::{ensure, Context, Result};
use re_cdr::{BigEndian, ByteOrder, CdrReader, LittleEndian};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CdrSchema {
    pub name: String,
    pub data: String,
    #[serde(default)]
    pub timestamp_prefix: Option<Layout>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Layout {
    Atom { alignment: usize, bytes: usize },
    String,
    Struct { fields: Vec<Layout> },
    Array { length: usize, element: Box<Layout> },
    Sequence { element: Box<Layout> },
    Stamp,
    Seconds { scalar: Scalar },
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Scalar {
    Bool,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
}

#[derive(Clone, Copy)]
struct SourceTime {
    seconds: f64,
    nanos: Option<u64>,
}

impl Scalar {
    fn read<BO: ByteOrder>(self, reader: &mut CdrReader<'_, BO>) -> Result<f64> {
        Ok(match self {
            Self::Bool => f64::from(u8::from(reader.read_bool()?)),
            Self::I8 => f64::from(reader.read_i8()?),
            Self::U8 => f64::from(reader.read_u8()?),
            Self::I16 => f64::from(reader.read_i16()?),
            Self::U16 => f64::from(reader.read_u16()?),
            Self::I32 => f64::from(reader.read_i32()?),
            Self::U32 => f64::from(reader.read_u32()?),
            Self::I64 => reader.read_i64()? as f64,
            Self::U64 => reader.read_u64()? as f64,
            Self::F32 => f64::from(reader.read_f32()?),
            Self::F64 => reader.read_f64()?,
        })
    }
}

impl CdrSchema {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.name.split('/').count() == 3 && self.name.contains("/msg/"),
            "CDR schema requires package/msg/Type identity"
        );
        ensure!(
            !self.data.trim().is_empty(),
            "CDR recording requires complete ros2msg schema data"
        );
        if let Some(prefix) = &self.timestamp_prefix {
            ensure!(
                prefix.validate()? == 1,
                "CDR timestamp projection must contain exactly one stamp"
            );
        }
        Ok(())
    }
}

impl Layout {
    fn validate(&self) -> Result<usize> {
        Ok(match self {
            Self::Atom { alignment, bytes } => {
                ensure!(
                    [1, 2, 4, 8].contains(alignment) && *bytes > 0 && bytes % alignment == 0,
                    "Invalid CDR primitive span"
                );
                0
            }
            Self::String => 0,
            Self::Stamp | Self::Seconds { .. } => 1,
            Self::Struct { fields } => fields.iter().map(Self::validate).sum::<Result<usize>>()?,
            Self::Array { element, .. } | Self::Sequence { element } => {
                ensure!(
                    element.validate()? == 0,
                    "Timestamp cannot be inside a repeated field"
                );
                0
            }
        })
    }

    fn empty(&self) -> bool {
        match self {
            Self::Struct { fields } => fields.iter().all(Self::empty),
            Self::Array { length, element } => *length == 0 || element.empty(),
            _ => false,
        }
    }

    fn read<BO: ByteOrder>(&self, reader: &mut CdrReader<'_, BO>) -> Result<Option<SourceTime>> {
        match self {
            Self::Atom { alignment, bytes } => {
                reader.align(*alignment)?;
                reader.read_bytes(*bytes)?;
            }
            Self::String => {
                reader.read_str()?;
            }
            Self::Stamp => {
                let sec = reader.read_i32()?;
                let nanosec = reader.read_u32()?;
                return Ok(Some(SourceTime {
                    seconds: f64::from(sec) + f64::from(nanosec) / 1_000_000_000.0,
                    nanos: u64::try_from(sec)
                        .ok()
                        .map(|sec| sec * 1_000_000_000 + u64::from(nanosec)),
                }));
            }
            Self::Seconds { scalar } => {
                return Ok(Some(SourceTime {
                    seconds: scalar.read(reader)?,
                    nanos: None,
                }))
            }
            Self::Struct { fields } => {
                for field in fields {
                    if let Some(stamp) = field.read(reader)? {
                        return Ok(Some(stamp));
                    }
                }
            }
            Self::Array { length, element } => element.read_array(*length, reader)?,
            Self::Sequence { element } => {
                let length = reader.read_sequence_length()?;
                element.read_array(length, reader)?;
            }
        }
        Ok(None)
    }

    fn read_array<BO: ByteOrder>(
        &self,
        length: usize,
        reader: &mut CdrReader<'_, BO>,
    ) -> Result<()> {
        if length == 0 || self.empty() {
            return Ok(());
        }
        if let Self::Atom { alignment, bytes } = self {
            reader.align(*alignment)?;
            let count = length
                .checked_mul(*bytes)
                .context("CDR prefix array size overflow")?;
            reader.read_bytes(count)?;
        } else {
            for _ in 0..length {
                self.read(reader)?;
            }
        }
        Ok(())
    }
}

/// Source seconds for Mem2 plus exact nanoseconds for standard MCAP publish_time.
pub(crate) fn timestamp(
    schema: &CdrSchema,
    data: &[u8],
    reception: f64,
) -> Result<(f64, Option<u64>)> {
    ensure!(
        data.len() >= 4 && data[0] == 0 && data[1] <= 1 && data[2] == 0 && data[3] == 0,
        "Expected plain CDR/XCDR1 encapsulation"
    );
    let Some(prefix) = &schema.timestamp_prefix else {
        return Ok((reception, None));
    };
    let stamp = if data[1] == 1 {
        prefix.read(&mut CdrReader::<LittleEndian>::new(&data[4..]))?
    } else {
        prefix.read(&mut CdrReader::<BigEndian>::new(&data[4..]))?
    }
    .context("CDR timestamp projection did not reach its stamp")?;
    Ok((stamp.seconds, stamp.nanos))
}

#[cfg(test)]
mod tests {
    use super::*;
    use re_cdr::CdrWriter;

    fn schema() -> CdrSchema {
        CdrSchema {
            name: "test_msgs/msg/StampedValues".into(),
            data: "string label\nfloat64[] values\nint32 sec\nuint32 nanosec\n".into(),
            timestamp_prefix: Some(Layout::Struct {
                fields: vec![
                    Layout::String,
                    Layout::Sequence {
                        element: Box::new(Layout::Atom {
                            alignment: 8,
                            bytes: 8,
                        }),
                    },
                    Layout::Stamp,
                ],
            }),
        }
    }

    fn packet<BO: ByteOrder>(little: bool, count: usize, sec: i32, nanosec: u32) -> Vec<u8> {
        let mut body = Vec::new();
        let mut writer = CdrWriter::<BO>::new(&mut body);
        writer.write_string("prefix-long");
        writer.write_sequence_length(count);
        for index in 0..count {
            writer.write_f64(index as f64 + 1.5);
        }
        writer.write_i32(sec);
        writer.write_u32(nanosec);
        let mut data = vec![0, u8::from(little), 0, 0];
        data.extend(body);
        data
    }

    #[test]
    fn projects_late_stamp_in_both_orders_and_empty_aligned_sequences() {
        let schema = schema();
        schema.validate().unwrap();
        for count in [0, 2] {
            for data in [
                packet::<LittleEndian>(true, count, 1_700_000_000, 123_456_789),
                packet::<BigEndian>(false, count, 1_700_000_000, 123_456_789),
            ] {
                let original = data.clone();
                let actual = timestamp(&schema, &data, 99.0).unwrap();
                assert_eq!(actual.1, Some(1_700_000_000_123_456_789));
                assert_eq!(actual.0, 1_700_000_000.0 + 123_456_789.0 / 1e9);
                assert_eq!(data, original);
            }
        }
        assert_eq!(
            timestamp(&schema, &packet::<LittleEndian>(true, 0, 0, 0), 99.0).unwrap(),
            (0.0, Some(0))
        );
    }

    #[test]
    fn rejects_bad_encapsulation_truncation_and_ambiguous_projection() {
        let mut data = packet::<LittleEndian>(true, 2, 17, 23);
        assert!(timestamp(&schema(), &data[..data.len() - 1], 99.0).is_err());
        data[1] = 3;
        assert!(timestamp(&schema(), &data, 99.0).is_err());
        let mut invalid = schema();
        invalid.timestamp_prefix = Some(Layout::Sequence {
            element: Box::new(Layout::Stamp),
        });
        assert!(invalid.validate().is_err());
        invalid.timestamp_prefix = Some(Layout::Atom {
            alignment: 0,
            bytes: 1,
        });
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn numeric_ts_preserves_zero_and_unstamped_values_use_reception() {
        let mut schema = CdrSchema {
            name: "test_msgs/msg/Event".into(),
            data: "float64 ts\n".into(),
            timestamp_prefix: Some(Layout::Seconds {
                scalar: Scalar::F64,
            }),
        };
        let mut body = Vec::new();
        CdrWriter::<LittleEndian>::new(&mut body).write_f64(0.0);
        let mut data = vec![0, 1, 0, 0];
        data.extend(body);
        assert_eq!(timestamp(&schema, &data, 99.0).unwrap(), (0.0, None));
        schema.timestamp_prefix = None;
        assert_eq!(timestamp(&schema, &data, 99.0).unwrap(), (99.0, None));
    }
}
