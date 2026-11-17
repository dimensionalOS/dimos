import assert from "node:assert/strict";

// Integration tests: run `cli.ts build` and install Playwright Chromium first.
// Use the empty scene so shutdown does not depend on downloaded robot assets.
for (const readyLine of ["[headless] Launching:", "Press Ctrl+C to stop."]) {
  for (const channels of [1, 2]) {
    for (const signal of ["SIGTERM", "SIGINT"] as const) {
      Deno.test(`dev with ${channels} page(s) exits promptly on ${signal} at ${readyLine}`, async () => {
        const listener = Deno.listen({ hostname: "127.0.0.1", port: 0 });
        const port = (listener.addr as Deno.NetAddr).port;
        listener.close();
        const child = new Deno.Command(Deno.execPath(), {
          args: [
            "run",
            "--allow-all",
            "--unstable-net",
            new URL("./cli.ts", import.meta.url).pathname,
            "dev",
            "--scene",
            "empty",
            "--headless",
            "--render",
            "cpu",
            "--port",
            String(port),
            "--channels",
            String(channels),
          ],
          stdout: "piped",
          stderr: "inherit",
        }).spawn();
        let exited = false;
        const status = child.status.then((result) => {
          exited = true;
          return result;
        });
        let timeout = setTimeout(() => child.kill("SIGKILL"), 120_000);
        const reader = child.stdout.pipeThrough(new TextDecoderStream())
          .getReader();
        let output = "";
        try {
          const marker = channels === 2 && readyLine === "[headless] Launching:"
            ? "[headless] Page 0: loading..."
            : readyLine;
          while (!output.includes(marker)) {
            const { value, done } = await reader.read();
            assert.equal(done, false, `CLI exited before ready: ${output}`);
            output += value;
          }
          const started = performance.now();
          clearTimeout(timeout);
          timeout = setTimeout(() => child.kill("SIGKILL"), 2000);
          child.kill(signal);
          // Keep draining stdout while shutdown closes Chromium.
          while (!(await reader.read()).done) { /* drain */ }
          const result = await status;
          assert.equal(result.code, 0, `CLI failed on ${signal}`);
          assert.ok(
            performance.now() - started < 1500,
            "shutdown exceeded 1.5 seconds",
          );
          // The bridge's listening socket must be released as well.
          const probe = Deno.listen({ hostname: "127.0.0.1", port });
          probe.close();
        } finally {
          clearTimeout(timeout);
          if (!exited) child.kill("SIGKILL");
          await status;
          await reader.cancel();
          reader.releaseLock();
        }
      });
    }
  }
}
