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

//! The shim's live stream: bind the unix datagram socket it streams to, receive records. Receive-only.

use std::os::fd::AsRawFd;
use std::os::unix::net::UnixDatagram;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result};

use crate::frames::{decode_tap_datagram, TapRecord};

pub struct TapReader {
    socket: UnixDatagram,
    path: PathBuf,
    buf: Vec<u8>,
}

impl TapReader {
    /// Bind the stream's path (a stale socket file from a reader that died is replaced).
    pub fn bind(path: &str, timeout: Duration) -> Result<TapReader> {
        let path = PathBuf::from(path);
        if path.exists() {
            std::fs::remove_file(&path)
                .with_context(|| format!("removing stale {}", path.display()))?;
        }
        let socket =
            UnixDatagram::bind(&path).with_context(|| format!("binding {}", path.display()))?;
        socket.set_read_timeout(Some(timeout))?;
        Ok(TapReader {
            socket,
            path,
            buf: vec![0u8; 1 << 16],
        })
    }

    pub fn fd(&self) -> i32 {
        self.socket.as_raw_fd()
    }

    /// The records of the next datagram, or none on a timeout / signal.
    pub fn recv(&mut self) -> Vec<TapRecord> {
        match self.socket.recv(&mut self.buf) {
            Ok(n) => decode_tap_datagram(&self.buf[..n]),
            Err(_) => Vec::new(),
        }
    }
}

impl Drop for TapReader {
    fn drop(&mut self) {
        // the shim stops streaming once the socket file is gone
        let _ = std::fs::remove_file(&self.path);
    }
}
