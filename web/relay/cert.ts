// Ephemeral self-signed certificate for WebTransport serverCertificateHashes.
// Chrome's constraints for hash-pinned certs: ECDSA P-256 and validity under
// 14 days. The browser pins the SHA-256 of the DER cert (via /api/info) and
// skips normal chain/hostname verification entirely.
import "reflect-metadata"; // @peculiar/x509 2.x needs the polyfill (tsyringe)
import * as x509 from "@peculiar/x509";

export interface EphemeralCert {
  certPem: string;
  keyPem: string;
  /** base64 SHA-256 of the DER cert, served via /api/info. */
  certHashB64: string;
}

export async function makeEphemeralCert(): Promise<EphemeralCert> {
  const alg = { name: "ECDSA", namedCurve: "P-256", hash: "SHA-256" };
  const keys = await crypto.subtle.generateKey(alg, true, ["sign", "verify"]);
  const cert = await x509.X509CertificateGenerator.createSelfSigned({
    serialNumber: Date.now().toString(16),
    name: "CN=localhost",
    notBefore: new Date(Date.now() - 3600_000), // 1 h clock-skew slack
    notAfter: new Date(Date.now() + 9 * 86400_000), // 9 days, under Chrome's 14-day cap
    signingAlgorithm: alg,
    keys,
    extensions: [
      new x509.SubjectAlternativeNameExtension([
        { type: "dns", value: "localhost" },
        { type: "ip", value: "127.0.0.1" },
      ]),
      new x509.BasicConstraintsExtension(false, undefined, true),
    ],
  });
  const der = new Uint8Array(cert.rawData);
  const hash = new Uint8Array(await crypto.subtle.digest("SHA-256", der));
  const pkcs8 = await crypto.subtle.exportKey("pkcs8", keys.privateKey);
  return {
    certPem: cert.toString("pem"),
    keyPem: x509.PemConverter.encode(pkcs8, "PRIVATE KEY"),
    certHashB64: btoa(String.fromCharCode(...hash)),
  };
}
