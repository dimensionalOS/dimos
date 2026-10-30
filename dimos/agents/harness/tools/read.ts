import type { AgentTool } from "./base.ts";
import type { ImageProcessor } from "@earendil-works/pi-durable/tools";
import type { ToolExecutionResult, ToolHookCall } from "@earendil-works/pi-durable";
import { agentTool } from "./lib/tool.ts";
import { createReadTool } from "@earendil-works/pi-durable/tools";
import { isAbsolute, resolve, sep } from "node:path";

/**
 * pi-durable's `read`, declared replay-safe (it declares nothing, which means unsafe), with images shrunk to fit.
 * A file under one of `wholeRoots` (the skill roots) is returned whole unless the call passes `max_chars`.
 */
export function readTool(images: ImageProcessor, wholeRoots: readonly string[]): AgentTool<"read"> {
  return agentTool("read", createReadTool({ images }), {
    replay: "safe",
    background: null,
    settle: null,
    fullTextAt: readSource,
    wholeResult: (call) => {
      const path = call.arguments["path"];
      return typeof path === "string" && wholeRoots.some((root) => resolve(path).startsWith(resolve(root) + sep));
    },
  });
}

/** A text read of an absolute path: the file itself holds the whole text. A relative path cannot be resolved here, so it is copied. */
function readSource(result: ToolExecutionResult, call: ToolHookCall): string | null {
  if (result.isError === true || result.output?.some((block) => block.type === "image")) return null;
  const path = call.arguments["path"];
  return typeof path === "string" && isAbsolute(path) ? path : null;
}
