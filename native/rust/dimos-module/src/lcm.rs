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

use dimos_lcm::{Lcm, LcmOptions};

use crate::transport::Transport;

/// LCM UDP multicast transport. Wraps `dimos_lcm::Lcm`.
pub struct LcmTransport(Lcm);

impl LcmTransport {
    pub async fn new() -> io::Result<Self> {
        Ok(Self(Lcm::new().await?))
    }

    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        Ok(Self(Lcm::with_options(opts).await?))
    }
}

impl Transport for LcmTransport {
    async fn publish(&self, channel: &str, data: &[u8]) -> io::Result<()> {
        self.0.publish(channel, data).await
    }

    async fn recv(&self) -> io::Result<(String, Vec<u8>)> {
        let msg = self.0.recv().await?;
        Ok((msg.channel, msg.data))
    }
}
