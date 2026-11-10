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

//! The dimos CBOR wire (`dimos/protocol/encode/cbor.py`) through serde.
//!
//! Python's tuple and dict tags are skipped on the way in, so an envelope
//! deserializes into ordinary Rust types. A dimos message, the Python class
//! plus LCM bytes under tag 4047, is the one form serde has no type for.

use std::collections::HashSet;
use std::sync::OnceLock;

use ciborium::tag::Required;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_bytes::ByteBuf;

pub type EncodeError = ciborium::ser::Error<std::io::Error>;
pub type DecodeError = ciborium::de::Error<std::io::Error>;

pub fn encode<T: Serialize + ?Sized>(value: &T) -> Result<Vec<u8>, EncodeError> {
    let mut buf = Vec::new();
    ciborium::into_writer(value, &mut buf)?;
    Ok(buf)
}

pub fn decode<T: DeserializeOwned>(data: &[u8]) -> Result<T, DecodeError> {
    ciborium::from_reader(data)
}

const TAG_DIMOSMSG: u64 = 4047;

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
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "Tagged", into = "Tagged")]
pub struct Msg {
    pub python_ref: String,
    pub data: Vec<u8>,
}

#[derive(Serialize, Deserialize)]
struct Tagged(Required<(String, ByteBuf), TAG_DIMOSMSG>);

impl From<Tagged> for Msg {
    fn from(Tagged(Required((python_ref, data))): Tagged) -> Self {
        Self {
            python_ref,
            data: data.into_vec(),
        }
    }
}

impl From<Msg> for Tagged {
    fn from(msg: Msg) -> Self {
        Tagged(Required((msg.python_ref, ByteBuf::from(msg.data))))
    }
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

#[cfg(test)]
pub(crate) fn fixture(name: &str) -> Vec<u8> {
    let table: serde_json::Map<String, serde_json::Value> =
        serde_json::from_str(include_str!("../../tests/fixtures/rpc_cbor.json")).unwrap();
    hex::decode(table[name].as_str().unwrap()).unwrap()
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use lcm_msgs::actionlib_msgs::GoalStatus;
    use lcm_msgs::geometry_msgs::PoseStamped;

    use super::*;

    /// The vectors `dimos/protocol/encode/test_cbor_wire.py` writes.
    #[test]
    fn python_payloads_decode_into_rust_types() {
        let (positional, keyword): ((f64, String), HashMap<String, f64>) =
            decode(&fixture("rpc_arguments")).unwrap();
        assert_eq!(positional, (1.0, "goal".to_owned()));
        assert_eq!(keyword, HashMap::from([("timeout".to_owned(), 2.5)]));

        let msg: Msg = decode(&fixture("msg_wrapped")).unwrap();
        assert_eq!(msg, Msg::new(PoseStamped::NAME, msg.data.clone()));
        let pose = PoseStamped::decode(&msg.data).unwrap();
        assert_eq!(pose.header.frame_id, "map");
        assert_eq!(pose.pose.position.y, 2.0);
        assert_eq!(decode::<Msg>(&encode(&msg).unwrap()).unwrap(), msg);

        let status: Msg = decode(&fixture("msg_lcm_only")).unwrap();
        assert_eq!(status, Msg::new(GoalStatus::NAME, status.data.clone()));
        assert_eq!(GoalStatus::decode(&status.data).unwrap().text, "active");
    }
}
