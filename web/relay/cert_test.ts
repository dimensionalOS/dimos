import { assert, assertEquals } from "@std/assert";
import "reflect-metadata"; // must precede @peculiar/x509 (tsyringe polyfill)
import * as x509 from "@peculiar/x509";
import { makeEphemeralCert } from "./cert.ts";

Deno.test("ephemeral cert: P-256, short validity, SANs, matching hash", async () => {
  const cert = await makeEphemeralCert();
  const parsed = new x509.X509Certificate(cert.certPem);

  const alg = parsed.publicKey.algorithm as { name: string; namedCurve?: string };
  assertEquals(alg.name, "ECDSA");
  assertEquals(alg.namedCurve, "P-256");

  // Chrome accepts serverCertificateHashes only for certs valid < 14 days.
  const validityDays = (parsed.notAfter.getTime() - parsed.notBefore.getTime()) / 86400_000;
  assert(validityDays < 14, `validity ${validityDays} days`);
  assert(parsed.notBefore.getTime() <= Date.now() - 3599_000, "clock-skew slack missing");
  assert(parsed.notAfter.getTime() > Date.now() + 8 * 86400_000, "expires too soon");

  const san = parsed.getExtension(x509.SubjectAlternativeNameExtension);
  const sanEntries = san ? san.names.items.map((n) => `${n.type}:${n.value}`) : [];
  assert(sanEntries.includes("dns:localhost"), sanEntries.join(","));
  assert(sanEntries.includes("ip:127.0.0.1"), sanEntries.join(","));

  const hash = new Uint8Array(
    await crypto.subtle.digest("SHA-256", new Uint8Array(parsed.rawData)),
  );
  assertEquals(btoa(String.fromCharCode(...hash)), cert.certHashB64);
  assertEquals(cert.certHashB64.length, 44);
  assert(cert.keyPem.includes("PRIVATE KEY"));
});
