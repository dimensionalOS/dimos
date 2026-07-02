// Source files must be plain text: a stray control byte (e.g. a literal NUL where a
// backslash-escape was intended) makes git treat the file as binary — no diffs, no grep,
// invisible in review — which is exactly why a test has to catch it.
import assert from "node:assert/strict";

const OK = new Set([0x09, 0x0a, 0x0d]); // tab, LF, CR

async function* walk(dir: URL): AsyncGenerator<URL> {
  for await (const e of Deno.readDir(dir)) {
    const child = new URL(e.isDirectory ? `${e.name}/` : e.name, dir);
    if (e.isDirectory) yield* walk(child);
    else if (e.name.endsWith(".ts") || e.name.endsWith(".tsx")) yield child;
  }
}

Deno.test("no control bytes in source files", async () => {
  const root = new URL(".", import.meta.url);
  let checked = 0;
  for await (const f of walk(root)) {
    const bytes = await Deno.readFile(f);
    checked++;
    for (let i = 0; i < bytes.length; i++) {
      const c = bytes[i];
      assert.ok(
        c >= 0x20 || OK.has(c),
        `${f.pathname}: control byte 0x${c.toString(16)} at offset ${i} (line ${
          new TextDecoder().decode(bytes.subarray(0, i)).split("\n").length
        })`,
      );
    }
  }
  assert.ok(checked > 10, `walked only ${checked} files — wrong root?`);
});
