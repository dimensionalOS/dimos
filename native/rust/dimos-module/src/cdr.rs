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

//! XCDR1 transport framing around the upstream Serde CDR codec.
use re_cdr::{BigEndian, LittleEndian};
use serde::{de::DeserializeOwned, Serialize};
use std::io;

pub fn encode<T: Serialize>(message: &T) -> io::Result<Vec<u8>> {
    encode_endian(message, true)
}

pub fn encode_endian<T: Serialize>(message: &T, little_endian: bool) -> io::Result<Vec<u8>> {
    let body = if little_endian {
        re_cdr::to_vec::<_, LittleEndian>(message)
    } else {
        re_cdr::to_vec::<_, BigEndian>(message)
    }
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error.to_string()))?;
    let mut bytes = Vec::with_capacity(4 + body.len());
    bytes.extend_from_slice(&[0, u8::from(little_endian), 0, 0]);
    bytes.extend(body);
    Ok(bytes)
}

pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> io::Result<T> {
    if bytes.len() < 4 || bytes[0] != 0 || bytes[1] > 1 || bytes[2] != 0 || bytes[3] != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Expected plain CDR/XCDR1 encapsulation",
        ));
    }
    let (value, consumed) = if bytes[1] == 1 {
        re_cdr::from_bytes::<T, LittleEndian>(&bytes[4..])
    } else {
        re_cdr::from_bytes::<T, BigEndian>(&bytes[4..])
    }
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error.to_string()))?;
    if consumed != bytes.len() - 4 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Trailing bytes after CDR message",
        ));
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct Int32 {
        data: i32,
    }

    #[test]
    fn literal_bytes_match_ros_cdr_in_both_byte_orders() {
        let value = Int32 { data: 42 };
        assert_eq!(encode(&value).unwrap(), vec![0, 1, 0, 0, 42, 0, 0, 0]);
        assert_eq!(
            encode_endian(&value, false).unwrap(),
            vec![0, 0, 0, 0, 0, 0, 0, 42]
        );
        assert_eq!(decode::<Int32>(&[0, 1, 0, 0, 42, 0, 0, 0]).unwrap(), value);
        assert_eq!(decode::<Int32>(&[0, 0, 0, 0, 0, 0, 0, 42]).unwrap(), value);
    }

    #[test]
    fn reject_wrong_encapsulation_truncation_and_trailing_bytes() {
        for bytes in [
            vec![],
            vec![0, 1, 0, 0, 42],
            vec![0, 3, 0, 0, 42, 0, 0, 0],
            vec![0, 1, 0, 0, 42, 0, 0, 0, 0],
        ] {
            assert_eq!(
                decode::<Int32>(&bytes).unwrap_err().kind(),
                io::ErrorKind::InvalidData
            );
        }
    }
}
