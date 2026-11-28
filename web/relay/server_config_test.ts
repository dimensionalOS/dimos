import { assertRejects } from "@std/assert";
import { startRelay } from "./server.ts";

for (const option of ["serveDir", "cockpitDir", "sdkDir"] as const) {
  Deno.test("startRelay rejects an empty " + option + " before binding", async () => {
    // The unpaired cert fails later, so even a broken directory check never opens a socket.
    await assertRejects(() => startRelay({ [option]: "", cert: "dummy" }), Error, option);
  });
}

Deno.test("startRelay rejects invalid HTTP ports", async () => {
  for (const port of [-1, 65536, 1.5]) {
    await assertRejects(() => startRelay({ port }), RangeError, "--port");
  }
});
