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

use std::io;

use re_cdr::{BigEndian, LittleEndian};
use serde::{de::DeserializeOwned, Serialize};

pub trait Message: Serialize + DeserializeOwned {
    const NAME: &'static str;
    const SCHEMA: &'static str;

    fn validate(&self) -> Result<(), String>;

    fn encode(&self) -> io::Result<Vec<u8>> {
        self.encode_endian(true)
    }

    fn encode_endian(&self, little_endian: bool) -> io::Result<Vec<u8>> {
        self.validate().map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
        let body = if little_endian {
            re_cdr::to_vec::<_, LittleEndian>(self)
        } else {
            re_cdr::to_vec::<_, BigEndian>(self)
        }.map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error.to_string()))?;
        let mut bytes = Vec::with_capacity(4 + body.len());
        bytes.extend_from_slice(&[0, u8::from(little_endian), 0, 0]);
        bytes.extend(body);
        Ok(bytes)
    }

    fn decode(bytes: &[u8]) -> io::Result<Self> {
        if bytes.len() < 4 || bytes[0] != 0 || bytes[1] > 1 || bytes[2] != 0 || bytes[3] != 0 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Expected plain CDR/XCDR1 encapsulation"));
        }
        let (value, consumed): (Self, usize) = if bytes[1] == 1 {
            re_cdr::from_bytes::<Self, LittleEndian>(&bytes[4..])
        } else {
            re_cdr::from_bytes::<Self, BigEndian>(&bytes[4..])
        }.map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error.to_string()))?;
        if consumed != bytes.len() - 4 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Trailing bytes after CDR message"));
        }
        value.validate().map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        Ok(value)
    }
}
