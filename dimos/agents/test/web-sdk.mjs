import { registerHooks } from "node:module";
import { webAliases } from "#dimos/agents/web-aliases.mjs";
import { readFileSync } from "node:fs";
import { transformSync } from "esbuild";

registerHooks({
  resolve(specifier, context, nextResolve) {
    const alias = webAliases[specifier];
    return alias === undefined ? nextResolve(specifier, context) : { url: alias.href, shortCircuit: true };
  },
  load(url, context, nextLoad) {
    if (!url.startsWith(new URL("../../../web/", import.meta.url).href) || !url.endsWith(".ts")) {
      return nextLoad(url, context);
    }
    return {
      format: "module",
      shortCircuit: true,
      source: transformSync(readFileSync(new URL(url), "utf8"), { loader: "ts", format: "esm", target: "node24" }).code,
    };
  },
});
