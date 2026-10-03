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

use dimos_lcm_transport::{Lcm, LcmOptions};
use std::io::{self, Write};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args()
        .nth(1)
        .ok_or("usage: interop <port>")?
        .parse()?;
    let transport = Lcm::with_options(LcmOptions {
        port,
        ttl: 0,
        ..Default::default()
    })
    .await?;
    println!("READY");
    io::stdout().flush()?;
    for _ in 0..3 {
        let message = tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                let message = transport.recv().await?;
                if message.channel == "INPUT" {
                    return Ok::<_, io::Error>(message);
                }
            }
        })
        .await??;
        transport.publish("REPLY", &message.data).await?;
    }
    Ok(())
}
