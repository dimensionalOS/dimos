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

    async fn recv(&mut self) -> io::Result<(String, Vec<u8>)> {
        let msg = self.0.recv().await?;
        Ok((msg.channel, msg.data))
    }
}
