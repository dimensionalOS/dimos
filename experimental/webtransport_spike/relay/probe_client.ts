// Debug probe: Deno's own WT client -> our relay, sends uni streams + a bidi
// stream as the "robot", to isolate server-side stream intake from aioquic.
// Run: deno run -A --unstable-net relay/probe_client.ts
const info = await (await fetch("http://localhost:8000/api/info")).json();
const hash = Uint8Array.from(atob(info.certHash), (c) => c.charCodeAt(0));
const wt = new WebTransport("https://127.0.0.1:4433/robot", {
  serverCertificateHashes: [{ algorithm: "sha-256", value: hash }],
});
wt.closed.then(
  (i) => console.log("[probe] closed cleanly:", JSON.stringify(i)),
  (e) => console.log("[probe] closed err:", (e as Error)?.message ?? e),
);
await wt.ready;
console.log("[probe] connected");

const enc = new TextEncoder();
function dataMsg(ch: string, seq: number, payload: Uint8Array): Uint8Array {
  const hdr = enc.encode(JSON.stringify({ ch, seq, ts: Date.now() / 1000 }));
  const out = new Uint8Array(8 + hdr.length + payload.length);
  const dv = new DataView(out.buffer);
  dv.setUint32(0, hdr.length, true);
  dv.setUint32(4, payload.length, true);
  out.set(hdr, 8);
  out.set(payload, 8 + hdr.length);
  return out;
}

// Variant A: write, wait, then close (FIN delayed)
{
  const s = await wt.createUnidirectionalStream();
  const w = s.getWriter();
  await w.write(dataMsg("probe_slow_fin", 0, enc.encode("delayed fin")));
  console.log("[probe] slow-fin uni written, holding open 1s");
  await new Promise((r) => setTimeout(r, 1000));
  await w.close();
  console.log("[probe] slow-fin uni closed");
}
// Variant B: write + close immediately (FIN with data)
{
  const s = await wt.createUnidirectionalStream();
  const w = s.getWriter();
  await w.write(dataMsg("probe_fast_fin", 0, enc.encode("immediate fin")));
  await w.close();
  console.log("[probe] fast-fin uni sent");
}
await new Promise((r) => setTimeout(r, 3000));
wt.close();
