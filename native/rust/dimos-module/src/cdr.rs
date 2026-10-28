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

//! SDK error and message-boundary adapter for the upstream complete CDR codec.
use ::cdr::{CdrBe, CdrLe, Infinite};
use serde::{de::DeserializeOwned, Serialize};
use std::io::{self, Cursor};

pub fn encode<T: Serialize>(message: &T) -> io::Result<Vec<u8>> {
    encode_endian(message, true)
}

pub fn encode_endian<T: Serialize>(message: &T, little_endian: bool) -> io::Result<Vec<u8>> {
    if little_endian {
        ::cdr::serialize::<_, _, CdrLe>(message, Infinite)
    } else {
        ::cdr::serialize::<_, _, CdrBe>(message, Infinite)
    }
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))
}

pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> io::Result<T> {
    // The SDK accepts standalone plain XCDR1, not parameter lists or other DDS representations.
    if !matches!(bytes.get(..4), Some([0, 0 | 1, 0, 0])) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Expected plain CDR/XCDR1 encapsulation",
        ));
    }
    let mut reader = Cursor::new(bytes);
    let value = ::cdr::deserialize_from(&mut reader, ::cdr::Bounded(bytes.len() as u64))
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    if reader.position() != bytes.len() as u64 {
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
    fn serialization_failure_is_an_error() {
        struct Unencodable;
        impl Serialize for Unencodable {
            fn serialize<S: serde::Serializer>(&self, _: S) -> Result<S::Ok, S::Error> {
                Err(serde::ser::Error::custom("invalid message"))
            }
        }
        assert_eq!(
            encode(&Unencodable).unwrap_err().kind(),
            io::ErrorKind::InvalidInput
        );
    }

    #[test]
    fn oversized_or_unterminated_strings_fail_at_the_library_boundary() {
        for bytes in [
            vec![0, 1, 0, 0, 255, 255, 255, 255],
            vec![0, 1, 0, 0, 0, 0, 0, 0],
            vec![0, 1, 0, 0, 2, 0, 0, 0, b'a', b'b'],
        ] {
            assert_eq!(
                decode::<String>(&bytes).unwrap_err().kind(),
                io::ErrorKind::InvalidData
            );
        }
        assert_eq!(
            decode::<String>(&encode(&"相机/café").unwrap()).unwrap(),
            "相机/café"
        );
    }

    #[test]
    fn reject_wrong_encapsulation_truncation_and_trailing_bytes() {
        for bytes in [
            vec![],
            vec![1, 1, 0, 0, 42, 0, 0, 0],
            vec![0, 1, 1, 0, 42, 0, 0, 0],
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
