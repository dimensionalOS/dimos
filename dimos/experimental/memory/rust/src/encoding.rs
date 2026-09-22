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

use std::io::Write;

use anyhow::{Context, Result};
use lz4_flex::frame::FrameEncoder;

use crate::decoding::DecodedObservation;
use crate::{Codec, StreamConfig};

#[derive(Debug)]
pub(crate) struct StoredObservation {
    pub(crate) ts: i64,
    pub(crate) data: Vec<u8>,
}

pub(crate) fn encode(
    stream: &StreamConfig,
    observation: DecodedObservation,
) -> Result<StoredObservation> {
    let data = match stream.codec {
        Codec::Cdr => observation.payload,
        Codec::Lz4Cdr => lz4_frame(&observation.payload)?,
    };
    Ok(StoredObservation {
        ts: observation.ts,
        data,
    })
}

pub(crate) fn lz4_frame(data: &[u8]) -> Result<Vec<u8>> {
    let mut encoder = FrameEncoder::new(Vec::new());
    encoder.write_all(data).context("LZ4 compression failed")?;
    encoder.finish().context("LZ4 frame finalization failed")
}
