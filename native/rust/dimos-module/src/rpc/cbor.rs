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

//! The portable subset of dimos' CBOR codec (`dimos/protocol/encode/cbor.py`).
//!
//! Primitives, lists, maps and three tagged forms cross between Python and
//! Rust: a tuple (tag 4040), a dict whose keys are not all primitives (tag
//! 4056) and a dimos message as its Python class plus LCM bytes (tag 4047).
//! Every other tag Python emits wraps a Python-only type, and decoding one is
//! an error that names it.
//!
//! Encoding reproduces Python's bytes exactly.

use std::collections::HashSet;
use std::sync::OnceLock;

use ciborium_io::Write;
use ciborium_ll::{Encoder, Header};
use thiserror::Error;

const TAG_TUPLE: u64 = 4040;
const TAG_DIMOSMSG: u64 = 4047;
const TAG_DICT: u64 = 4056;

/// The Python-only tags, by what they carry.
const PYTHON_ONLY_TAGS: &[(u64, &str)] = &[
    (4041, "NamedTuple"),
    (4042, "set"),
    (4043, "frozenset"),
    (4044, "numpy ndarray"),
    (4045, "numpy scalar"),
    (4046, "Enum"),
    (4048, "dataclass"),
    (4049, "pydantic model"),
    (4050, "Path"),
    (4051, "class"),
    (4052, "function"),
    (4053, "functools.partial"),
    (4054, "cbor_encode class"),
    (4055, "registered codec"),
    (4057, "generic alias"),
    (4058, "union"),
    (4059, "Ellipsis"),
    (4060, "Literal"),
];

/// LCM type names with a `dimos.msgs` wrapper class, which Python's `encode`
/// names instead of the `dimos_lcm` class.
const WRAPPED_MESSAGES: &str = include_str!("../../tests/fixtures/msg_wrappers.json");

fn wrapped_messages() -> &'static HashSet<&'static str> {
    static SET: OnceLock<HashSet<&'static str>> = OnceLock::new();
    SET.get_or_init(|| {
        serde_json::from_str::<Vec<&str>>(WRAPPED_MESSAGES)
            .expect("msg_wrappers.json is a list of LCM type names")
            .into_iter()
            .collect()
    })
}

/// A dimos message on the wire: the Python class that decodes it, as
/// `"module:qualname"`, and its LCM bytes.
#[derive(Clone, Debug, PartialEq)]
pub struct Msg {
    pub python_ref: String,
    pub data: Vec<u8>,
}

impl Msg {
    /// A message of the LCM type `name` (`"geometry_msgs.PoseStamped"`), as
    /// the generated types spell it in their `NAME` constant.
    pub fn new(name: &str, data: Vec<u8>) -> Self {
        let (package, type_name) = name
            .rsplit_once('.')
            .unwrap_or_else(|| panic!("{name:?} is not a package.Type LCM name"));
        let module = if wrapped_messages().contains(name) {
            "dimos.msgs"
        } else {
            "dimos_lcm"
        };
        Self {
            python_ref: format!("{module}.{package}.{type_name}:{type_name}"),
            data,
        }
    }
}

/// A value that both Python and Rust can hold. A CBOR integer outside `i64`
/// fails to decode.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    Bytes(Vec<u8>),
    List(Vec<Value>),
    Tuple(Vec<Value>),
    /// Entries in insertion order, as a Python dict keeps them.
    Map(Vec<(Value, Value)>),
    Msg(Msg),
}

impl Value {
    pub fn msg(name: &str, data: Vec<u8>) -> Self {
        Value::Msg(Msg::new(name, data))
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Value::Int(n) => Some(*n),
            _ => None,
        }
    }

    /// An `Int` counts as a float.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Float(f) => Some(*f),
            Value::Int(n) => Some(*n as f64),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_bytes(&self) -> Option<&[u8]> {
        match self {
            Value::Bytes(b) => Some(b),
            _ => None,
        }
    }

    /// The items of a list or a tuple.
    pub fn as_slice(&self) -> Option<&[Value]> {
        match self {
            Value::List(items) | Value::Tuple(items) => Some(items),
            _ => None,
        }
    }

    pub fn as_map(&self) -> Option<&[(Value, Value)]> {
        match self {
            Value::Map(entries) => Some(entries),
            _ => None,
        }
    }

    pub fn as_msg(&self) -> Option<&Msg> {
        match self {
            Value::Msg(msg) => Some(msg),
            _ => None,
        }
    }

    /// The value under a string key of a map.
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.as_map()?
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, v)| v)
    }

    /// Whether Python would accept this as a dict key without tagging the dict.
    fn is_primitive(&self) -> bool {
        matches!(
            self,
            Value::Null
                | Value::Bool(_)
                | Value::Int(_)
                | Value::Float(_)
                | Value::Str(_)
                | Value::Bytes(_)
        )
    }
}

impl From<bool> for Value {
    fn from(b: bool) -> Self {
        Value::Bool(b)
    }
}

impl From<i64> for Value {
    fn from(n: i64) -> Self {
        Value::Int(n)
    }
}

impl From<i32> for Value {
    fn from(n: i32) -> Self {
        Value::Int(n.into())
    }
}

impl From<f64> for Value {
    fn from(f: f64) -> Self {
        Value::Float(f)
    }
}

impl From<f32> for Value {
    fn from(f: f32) -> Self {
        Value::Float(f.into())
    }
}

impl From<&str> for Value {
    fn from(s: &str) -> Self {
        Value::Str(s.to_owned())
    }
}

impl From<String> for Value {
    fn from(s: String) -> Self {
        Value::Str(s)
    }
}

impl From<Vec<Value>> for Value {
    fn from(items: Vec<Value>) -> Self {
        Value::List(items)
    }
}

impl<T: Into<Value>> From<Option<T>> for Value {
    fn from(opt: Option<T>) -> Self {
        opt.map_or(Value::Null, Into::into)
    }
}

#[derive(Debug, Error)]
pub enum DecodeError {
    #[error("invalid CBOR: {0}")]
    Cbor(String),
    #[error("CBOR tag {tag} carries a Python {what}, which has no Rust form")]
    PythonOnly { tag: u64, what: &'static str },
    #[error("unknown CBOR tag {0}")]
    UnknownTag(u64),
    #[error("CBOR tag {0} has a malformed payload")]
    MalformedTag(u64),
    #[error("integer {0} does not fit an i64")]
    IntegerRange(i128),
}

/// The bytes Python's `encode` produces for the same value.
pub fn encode(value: &Value) -> Vec<u8> {
    let mut buf = Vec::new();
    write(&mut Encoder::from(&mut buf), value).expect("writing to a Vec");
    buf
}

type Out<'a> = Encoder<&'a mut Vec<u8>>;

fn write(out: &mut Out, value: &Value) -> std::io::Result<()> {
    match value {
        Value::Null => out.push(Header::Simple(ciborium_ll::simple::NULL)),
        Value::Bool(false) => out.push(Header::Simple(ciborium_ll::simple::FALSE)),
        Value::Bool(true) => out.push(Header::Simple(ciborium_ll::simple::TRUE)),
        Value::Int(n) if *n >= 0 => out.push(Header::Positive(*n as u64)),
        Value::Int(n) => out.push(Header::Negative(!*n as u64)),
        Value::Float(f) => write_float(out, *f),
        Value::Str(s) => out.text(s, None),
        Value::Bytes(b) => out.bytes(b, None),
        Value::List(items) => write_array(out, items),
        Value::Tuple(items) => {
            out.push(Header::Tag(TAG_TUPLE))?;
            write_array(out, items)
        }
        Value::Map(entries) if entries.iter().all(|(k, _)| k.is_primitive()) => {
            out.push(Header::Map(Some(entries.len())))?;
            for (key, value) in entries {
                write(out, key)?;
                write(out, value)?;
            }
            Ok(())
        }
        Value::Map(entries) => {
            out.push(Header::Tag(TAG_DICT))?;
            out.push(Header::Array(Some(entries.len())))?;
            for (key, value) in entries {
                out.push(Header::Array(Some(2)))?;
                write(out, key)?;
                write(out, value)?;
            }
            Ok(())
        }
        Value::Msg(msg) => {
            out.push(Header::Tag(TAG_DIMOSMSG))?;
            out.push(Header::Array(Some(2)))?;
            out.text(&msg.python_ref, None)?;
            out.bytes(&msg.data, None)
        }
    }
}

fn write_array(out: &mut Out, items: &[Value]) -> std::io::Result<()> {
    out.push(Header::Array(Some(items.len())))?;
    items.iter().try_for_each(|item| write(out, item))
}

/// cbor2 writes every finite float as a double and the three non-finite
/// values as half floats.
fn write_float(out: &mut Out, f: f64) -> std::io::Result<()> {
    if f.is_nan() {
        out.write_all(&[0xf9, 0x7e, 0x00])
    } else if f == f64::INFINITY {
        out.write_all(&[0xf9, 0x7c, 0x00])
    } else if f == f64::NEG_INFINITY {
        out.write_all(&[0xf9, 0xfc, 0x00])
    } else {
        out.write_all(&[0xfb])?;
        out.write_all(&f.to_be_bytes())
    }
}

/// The value in bytes Python's `encode` produced.
pub fn decode(data: &[u8]) -> Result<Value, DecodeError> {
    let raw: ciborium::Value =
        ciborium::from_reader(data).map_err(|e| DecodeError::Cbor(e.to_string()))?;
    convert(raw)
}

fn convert(raw: ciborium::Value) -> Result<Value, DecodeError> {
    use ciborium::Value as Raw;
    Ok(match raw {
        Raw::Null => Value::Null,
        Raw::Bool(b) => Value::Bool(b),
        Raw::Integer(n) => {
            let n = i128::from(n);
            Value::Int(i64::try_from(n).map_err(|_| DecodeError::IntegerRange(n))?)
        }
        Raw::Float(f) => Value::Float(f),
        Raw::Text(s) => Value::Str(s),
        Raw::Bytes(b) => Value::Bytes(b),
        Raw::Array(items) => Value::List(convert_all(items)?),
        Raw::Map(entries) => Value::Map(convert_pairs(entries)?),
        Raw::Tag(TAG_TUPLE, inner) => match *inner {
            Raw::Array(items) => Value::Tuple(convert_all(items)?),
            _ => return Err(DecodeError::MalformedTag(TAG_TUPLE)),
        },
        Raw::Tag(TAG_DICT, inner) => {
            let pairs = match *inner {
                Raw::Array(pairs) => pairs,
                _ => return Err(DecodeError::MalformedTag(TAG_DICT)),
            };
            let mut entries = Vec::with_capacity(pairs.len());
            for pair in pairs {
                let Raw::Array(pair) = pair else {
                    return Err(DecodeError::MalformedTag(TAG_DICT));
                };
                let [key, value]: [Raw; 2] = pair
                    .try_into()
                    .map_err(|_| DecodeError::MalformedTag(TAG_DICT))?;
                entries.push((convert(key)?, convert(value)?));
            }
            Value::Map(entries)
        }
        Raw::Tag(TAG_DIMOSMSG, inner) => {
            let Raw::Array(parts) = *inner else {
                return Err(DecodeError::MalformedTag(TAG_DIMOSMSG));
            };
            let [python_ref, data]: [Raw; 2] = parts
                .try_into()
                .map_err(|_| DecodeError::MalformedTag(TAG_DIMOSMSG))?;
            match (python_ref, data) {
                (Raw::Text(python_ref), Raw::Bytes(data)) => Value::Msg(Msg { python_ref, data }),
                _ => return Err(DecodeError::MalformedTag(TAG_DIMOSMSG)),
            }
        }
        Raw::Tag(tag, _) => {
            return Err(match PYTHON_ONLY_TAGS.iter().find(|(t, _)| *t == tag) {
                Some((_, what)) => DecodeError::PythonOnly { tag, what },
                None => DecodeError::UnknownTag(tag),
            })
        }
        other => {
            return Err(DecodeError::Cbor(format!(
                "unsupported CBOR item {other:?}"
            )))
        }
    })
}

fn convert_all(items: Vec<ciborium::Value>) -> Result<Vec<Value>, DecodeError> {
    items.into_iter().map(convert).collect()
}

fn convert_pairs(
    entries: Vec<(ciborium::Value, ciborium::Value)>,
) -> Result<Vec<(Value, Value)>, DecodeError> {
    entries
        .into_iter()
        .map(|(k, v)| Ok((convert(k)?, convert(v)?)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use lcm_msgs::actionlib_msgs::{GoalID, GoalStatus};
    use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
    use lcm_msgs::std_msgs::{Header, Time};

    /// The vectors `dimos/protocol/encode/test_cbor_wire.py` writes, by name.
    fn fixtures() -> Vec<(String, Vec<u8>)> {
        let table: serde_json::Map<String, serde_json::Value> =
            serde_json::from_str(include_str!("../../tests/fixtures/rpc_cbor.json")).unwrap();
        table
            .into_iter()
            .map(|(name, hex)| (name, hex::decode(hex.as_str().unwrap()).unwrap()))
            .collect()
    }

    fn pairs(entries: Vec<(Value, Value)>) -> Value {
        Value::Map(entries)
    }

    fn expected(name: &str) -> Value {
        match name {
            "null" => Value::Null,
            "true" => Value::Bool(true),
            "false" => Value::Bool(false),
            "int_zero" => Value::Int(0),
            "int_small" => Value::Int(1),
            "int_negative" => Value::Int(-1),
            "int_large" => Value::Int(1 << 40),
            "int_min" => Value::Int(i64::MIN),
            "int_max" => Value::Int(i64::MAX),
            "float_one" => Value::Float(1.0),
            "float_negative_zero" => Value::Float(-0.0),
            "float_subnormal" => Value::Float(5e-324),
            "float_large" => Value::Float(1e300),
            "float_nan" => Value::Float(f64::NAN),
            "float_inf" => Value::Float(f64::INFINITY),
            "float_neg_inf" => Value::Float(f64::NEG_INFINITY),
            "str" => Value::from("héllo"),
            "str_empty" => Value::from(""),
            "bytes" => Value::Bytes(vec![0x00, 0x01, 0xff]),
            "bytes_empty" => Value::Bytes(vec![]),
            "list" => Value::List(vec![Value::Int(1), Value::from("a"), Value::Null]),
            "list_empty" => Value::List(vec![]),
            "tuple" => Value::Tuple(vec![Value::Int(1), Value::Float(2.5)]),
            "tuple_empty" => Value::Tuple(vec![]),
            "dict_str_keys" => pairs(vec![
                (Value::from("a"), Value::Int(1)),
                (Value::from("b"), Value::List(vec![Value::Int(2)])),
            ]),
            "dict_empty" => pairs(vec![]),
            "dict_primitive_keys" => pairs(vec![
                (Value::Int(1), Value::from("a")),
                (Value::Null, Value::Int(2)),
                (Value::Bytes(b"k".to_vec()), Value::Bool(true)),
                (Value::Float(2.5), Value::Null),
            ]),
            "dict_tuple_keys" => pairs(vec![(
                Value::Tuple(vec![Value::Int(1), Value::Int(2)]),
                Value::from("t"),
            )]),
            "nested" => Value::Tuple(vec![
                Value::List(vec![pairs(vec![(
                    Value::from("k"),
                    Value::Tuple(vec![Value::Int(1), Value::Bytes(b"x".to_vec())]),
                )])]),
                pairs(vec![(
                    Value::from("m"),
                    pairs(vec![(Value::from("n"), Value::List(vec![Value::Null]))]),
                )]),
            ]),
            "rpc_arguments" => Value::Tuple(vec![
                Value::Tuple(vec![Value::Float(1.0), Value::from("goal")]),
                pairs(vec![(Value::from("timeout"), Value::Float(2.5))]),
            ]),
            "msg_wrapped" => {
                let pose = PoseStamped {
                    header: Header {
                        seq: 0,
                        stamp: Time {
                            sec: 1,
                            nsec: 500_000_000,
                        },
                        frame_id: "map".into(),
                    },
                    pose: Pose {
                        position: Point {
                            x: 1.0,
                            y: 2.0,
                            z: 3.0,
                        },
                        orientation: Quaternion {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 1.0,
                        },
                    },
                };
                Value::msg(PoseStamped::NAME, pose.encode())
            }
            "msg_lcm_only" => {
                let status = GoalStatus {
                    goal_id: GoalID {
                        stamp: Time { sec: 7, nsec: 8 },
                        id: "g1".into(),
                    },
                    status: 1,
                    text: "active".into(),
                };
                Value::msg(GoalStatus::NAME, status.encode())
            }
            other => panic!("the Python suite added a vector {other:?} without a Rust twin"),
        }
    }

    #[test]
    fn every_python_vector_decodes_and_re_encodes_byte_for_byte() {
        let fixtures = fixtures();
        assert!(
            fixtures.len() > 20,
            "the fixture table is suspiciously short"
        );
        for (name, bytes) in fixtures {
            let want = expected(&name);
            let got = decode(&bytes).unwrap_or_else(|e| panic!("{name}: {e}"));
            // NaN is not equal to itself; its bytes are checked below.
            if name != "float_nan" {
                assert_eq!(got, want, "{name}");
            }
            assert_eq!(hex::encode(encode(&want)), hex::encode(&bytes), "{name}");
        }
    }

    #[test]
    fn python_only_tags_are_named() {
        // Tag 4044 (an ndarray) around an empty array.
        let err = decode(&[0xd9, 0x0f, 0xcc, 0x80]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "CBOR tag 4044 carries a Python numpy ndarray, which has no Rust form"
        );
        let err = decode(&[0xd8, 0x63, 0x00]).unwrap_err();
        assert!(matches!(err, DecodeError::UnknownTag(99)), "{err}");
    }
}
