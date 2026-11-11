// Package build only. The root browser bundle stays one self-contained sdk.js;
// Node/bundler consumers get ESM entries and bundled declarations below package/.
import { fileURLToPath } from "node:url";
import { rollup } from "rollup";
import { dts } from "rollup-plugin-dts";
import { build } from "vite";
import browser from "./vite.config.ts";

const entries = {
  index: fileURLToPath(new URL("./src/index.ts", import.meta.url)),
  react: fileURLToPath(new URL("./src/react.ts", import.meta.url)),
  "internal/teleop": fileURLToPath(new URL("./src/internal/teleopMachine.ts", import.meta.url)),
  "internal/topics": fileURLToPath(new URL("./src/topics.ts", import.meta.url)),
};
const outDir = fileURLToPath(new URL("./dist/package", import.meta.url));
const external = ["react", "react/jsx-runtime"];

await build({
  ...browser,
  configFile: false,
  build: {
    target: "es2022",
    outDir,
    lib: { entry: entries, formats: ["es"], fileName: (_format, name) => `${name}.js` },
    rollupOptions: { external },
  },
});

const declarations = await rollup({
  input: entries,
  plugins: [dts({ tsconfig: fileURLToPath(new URL("./tsconfig.json", import.meta.url)) })],
  external,
});
try {
  await declarations.write({
    dir: outDir,
    format: "es",
    entryFileNames: "[name].d.ts",
    chunkFileNames: "[name]-[hash].d.ts",
  });
} finally {
  await declarations.close();
}
