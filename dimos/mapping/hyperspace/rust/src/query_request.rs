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

//! What arrives on the `query` topic.

use serde::Deserialize;

/// One request. `id` comes back on the answer's `header.seq`, which is what lets
/// a caller match answers to requests over plain pub/sub.
#[derive(Debug, Clone, PartialEq)]
pub struct QueryRequest {
    pub id: i32,
    pub text: String,
    /// Frame to answer in. Empty means the module's configured world frame.
    pub frame: String,
}

#[derive(Deserialize)]
struct Wire {
    #[serde(default)]
    id: i32,
    text: String,
    #[serde(default)]
    frame: String,
}

impl QueryRequest {
    /// JSON `{"id": 7, "text": "a chair", "frame": "odom"}`, or bare text for a
    /// caller that does not care about pairing (`dimos topic pub` by hand).
    pub fn parse(payload: &str) -> Result<Self, String> {
        let trimmed = payload.trim();
        if trimmed.is_empty() {
            return Err("empty query".into());
        }
        if !trimmed.starts_with('{') {
            return Ok(QueryRequest {
                id: 0,
                text: trimmed.to_string(),
                frame: String::new(),
            });
        }
        let wire: Wire =
            serde_json::from_str(trimmed).map_err(|e| format!("bad query json: {e}"))?;
        if wire.text.trim().is_empty() {
            return Err("query json has no text".into());
        }
        Ok(QueryRequest {
            id: wire.id,
            text: wire.text.trim().to_string(),
            frame: wire.frame,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_carries_the_request_id() {
        let request = QueryRequest::parse(r#"{"id": 7, "text": "a chair"}"#).unwrap();
        assert_eq!(
            request,
            QueryRequest {
                id: 7,
                text: "a chair".into(),
                frame: String::new()
            }
        );
    }

    #[test]
    fn frame_can_be_overridden() {
        let request =
            QueryRequest::parse(r#"{"id": 1, "text": "a door", "frame": "map"}"#).unwrap();
        assert_eq!(request.frame, "map");
    }

    #[test]
    fn bare_text_gets_id_zero() {
        let request = QueryRequest::parse("  a trash can  ").unwrap();
        assert_eq!(
            request,
            QueryRequest {
                id: 0,
                text: "a trash can".into(),
                frame: String::new()
            }
        );
    }

    #[test]
    fn empty_and_textless_payloads_are_errors() {
        assert!(QueryRequest::parse("   ").is_err());
        assert!(QueryRequest::parse(r#"{"id": 3}"#).is_err());
        assert!(QueryRequest::parse(r#"{"id": 3, "text": "  "}"#).is_err());
        assert!(QueryRequest::parse("{not json").is_err());
    }
}
