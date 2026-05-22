use std::io;
use std::net::Ipv4Addr;

use dimos_lcm::{Lcm, LcmOptions};

use crate::transport::Transport;

/// LCM UDP multicast transport. Wraps `dimos_lcm::Lcm`.
pub struct LcmTransport(Lcm);

impl LcmTransport {
    pub async fn new() -> io::Result<Self> {
        // Honor LCM_DEFAULT_URL the same way the python LCM client does, so a
        // Python parent and a Rust subprocess end up on the same multicast bus
        // when the env var is set (e.g. by `dimos/conftest.py` to isolate
        // pytest runs from co-running deployments). Without this, the Python
        // side would publish on the custom bus while the Rust subprocess
        // stayed on the default 239.255.76.67:7667 — silently dropping all
        // messages between them.
        match std::env::var("LCM_DEFAULT_URL") {
            Ok(url) => match parse_lcm_url(&url) {
                Some(opts) => Ok(Self(Lcm::with_options(opts).await?)),
                None => {
                    eprintln!("dimos_module: LCM_DEFAULT_URL={url:?} unparseable, using defaults");
                    Ok(Self(Lcm::new().await?))
                }
            },
            Err(_) => Ok(Self(Lcm::new().await?)),
        }
    }

    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        Ok(Self(Lcm::with_options(opts).await?))
    }
}

/// Parse the LCM URL format `udpm://<group>:<port>[?<k>=<v>&...]` into LcmOptions.
/// Recognized query keys: `ttl`, `interface`. Unknown keys are logged via stderr
/// so a typo (e.g. `tll=`) is visible instead of silent.
/// Returns None on malformed scheme/host/port; the caller falls back to defaults.
/// Note: multicast is IPv4 only in LCM, so `rsplit_once(':')` is safe here.
fn parse_lcm_url(url: &str) -> Option<LcmOptions> {
    let rest = url.strip_prefix("udpm://")?;
    let (host_port, query) = match rest.split_once('?') {
        Some((hp, q)) => (hp, Some(q)),
        None => (rest, None),
    };
    let (host, port_str) = host_port.rsplit_once(':')?;
    let multicast_group: Ipv4Addr = host.parse().ok()?;
    let port: u16 = port_str.parse().ok()?;

    let mut opts = LcmOptions {
        multicast_group,
        port,
        ..LcmOptions::default()
    };
    if let Some(q) = query {
        for kv in q.split('&') {
            let Some((k, v)) = kv.split_once('=') else {
                continue;
            };
            match k {
                "ttl" => {
                    if let Ok(ttl) = v.parse::<u32>() {
                        opts.ttl = ttl;
                    }
                }
                "interface" => {
                    if let Ok(iface) = v.parse::<Ipv4Addr>() {
                        opts.interface = iface;
                    }
                }
                _ => {
                    eprintln!("dimos_module: LCM_DEFAULT_URL: ignoring unknown query key {k:?}");
                }
            }
        }
    }
    Some(opts)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_full_url() {
        let opts = parse_lcm_url("udpm://239.255.99.99:8876?ttl=0").unwrap();
        assert_eq!(opts.multicast_group, Ipv4Addr::new(239, 255, 99, 99));
        assert_eq!(opts.port, 8876);
        assert_eq!(opts.ttl, 0);
    }

    #[test]
    fn parses_url_without_query() {
        let opts = parse_lcm_url("udpm://239.255.76.67:7667").unwrap();
        assert_eq!(opts.multicast_group, Ipv4Addr::new(239, 255, 76, 67));
        assert_eq!(opts.port, 7667);
    }

    #[test]
    fn parses_nonzero_ttl_and_interface() {
        let opts = parse_lcm_url("udpm://239.255.76.67:7667?ttl=5&interface=10.0.0.1").unwrap();
        assert_eq!(opts.ttl, 5);
        assert_eq!(opts.interface, Ipv4Addr::new(10, 0, 0, 1));
    }

    #[test]
    fn rejects_non_udpm_scheme() {
        assert!(parse_lcm_url("tcp://host:1234").is_none());
        assert!(parse_lcm_url("").is_none());
    }

    #[test]
    fn rejects_malformed() {
        assert!(parse_lcm_url("udpm://no-port").is_none());
        assert!(parse_lcm_url("udpm://239.255.76.67:notaport").is_none());
    }
}
