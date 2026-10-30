import { spawn } from "node:child_process";
import { glob } from "node:fs/promises";

const tests = [];
for await (const path of glob(["**/*.test.ts", "**/*.test.mjs"], {
  exclude: (name) => name === "node_modules" || name === "dist",
})) {
  tests.push(path);
}
const child = spawn(process.execPath, ["--test", ...process.argv.slice(2), ...tests.sort()], {
  stdio: "inherit",
  env: {
    ...process.env,
    NODE_OPTIONS: [process.env.NODE_OPTIONS, `--import=${new URL("./web-sdk.mjs", import.meta.url).href}`]
      .filter(Boolean)
      .join(" "),
  },
});
child.on("exit", (code) => {
  process.exitCode = code ?? 1;
});
