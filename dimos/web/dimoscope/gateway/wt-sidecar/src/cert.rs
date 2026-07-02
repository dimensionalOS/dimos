//! Self-signed identity in the shape Chrome's `serverCertificateHashes`
//! accepts: ECDSA P-256, 10-day validity, SHA-256-of-DER published as
//! lowercase hex. The gateway serves the hash via /cert.

use anyhow::{Context, Result};
use wtransport::tls::Identity;

pub struct CertBundle {
    pub identity: Identity,
    pub hash_hex: String,
}

pub fn make_identity() -> Result<CertBundle> {
    let identity = Identity::self_signed_builder()
        .subject_alt_names(["localhost", "127.0.0.1", "::1"])
        .from_now_utc()
        .validity_days(10)
        .build()
        .context("self-signed identity")?;
    let cert = identity
        .certificate_chain()
        .as_slice()
        .first()
        .context("empty certificate chain")?;
    // Certificate::hash() is SHA-256 of the DER — exactly what
    // serverCertificateHashes verifies (webTransport.ts hexToBytes).
    let hash_hex = hex_lower(cert.hash().as_ref());
    Ok(CertBundle { identity, hash_hex })
}

/// Write the hash where the gateway's /cert route reads it (WT_CERT_HASH_FILE).
pub fn write_hash_file(path: &str, hash_hex: &str) -> Result<()> {
    std::fs::write(path, hash_hex).with_context(|| format!("writing {path}"))
}

fn hex_lower(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hash_is_lowercase_hex_sha256() {
        let b = make_identity().unwrap();
        assert_eq!(b.hash_hex.len(), 64);
        assert!(b
            .hash_hex
            .chars()
            .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()));
    }
}
