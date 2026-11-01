import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { DetailsByTool } from "#dimos/agents/shared/protocol/tools/details.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import type { Static } from "typebox";
import type { ToolExecutionResult } from "@earendil-works/pi-durable";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { bashEnvironment } from "#dimos/agents/harness/env.ts";
import { createBashTool } from "@earendil-works/pi-durable/tools";
import { createReadStream } from "node:fs";
import { extname, join, resolve } from "node:path";
import { readdir, stat } from "node:fs/promises";
import { tmpdir } from "node:os";

type TruncationRecord = NonNullable<DetailsByTool["bash"]["truncation"]>;

type BashToolOptions = {
  readonly install: DimosInstall;
  /** The gateway's environment; `bashEnvironment` decides what of it bash sees. */
  readonly gatewayEnv: NodeJS.ProcessEnv;
  readonly renderer: FileRenderer;
};

/** What pi-durable's bash returns as `structuredOutput`; absent when the command timed out or could not start. */
const ShellOutput = Type.Object({
  output: Type.String(),
  truncated: Type.Boolean(),
  fullOutputPath: Type.Optional(Type.String()),
  exitCode: Type.Integer(),
});

/** Formats the renderers read: what `sharp` opens, plus point-cloud and frame-index JSON. */
const RENDERABLE = new Set([".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg", ".tif", ".tiff", ".avif", ".json"]);

/**
 * A run of path characters that ends in a renderable extension, after a separator. The second branch takes a run
 * that names no such file whole, so every run is scanned once whatever its slashes, `=` and `,`.
 */
const PATH_IN_OUTPUT =
  /(?:^|[\s"'`(=:,])(?:([^\s"'`()<>|:]+\.(?:png|jpe?g|gif|webp|svg|tiff?|avif|json))\b|[^\s"'`()<>|:]+)/gi;

const SKIPPED_DIRECTORIES = new Set(["node_modules", ".venv", "__pycache__", ".git"]);

/** Directory entries examined per call, over the working directory, one level down, and the temp directory. */
const SCAN_BUDGET = 2_000;

function shellOutput(result: ToolExecutionResult): Static<typeof ShellOutput> | null {
  return Value.Check(ShellOutput, result.structuredOutput) ? result.structuredOutput : null;
}

/** Absolute paths of renderable files named in the output, resolved against the command's directory. */
function namedIn(output: string, cwd: string): string[] {
  const found = new Set<string>();
  for (const match of output.matchAll(PATH_IN_OUTPUT)) {
    const path = match[1];
    if (path !== undefined) found.add(resolve(cwd, path));
  }
  return [...found];
}

/** A renderable file a scan found changed, with its modification time then. */
type Changed = { readonly path: string; readonly mtimeMs: number };

/** Renderable regular files under `directory` (and one level down when `depth` is 1) modified at or after `sinceMs`. */
async function changedUnder(
  directory: string,
  depth: 0 | 1,
  sinceMs: number,
  budget: { left: number },
): Promise<Changed[]> {
  const found: Changed[] = [];
  const entries = await readdir(directory, { withFileTypes: true }).catch(() => []);
  for (const entry of entries) {
    if (budget.left <= 0) break;
    budget.left -= 1;
    const path = join(directory, entry.name);
    if (entry.isDirectory()) {
      if (depth === 1 && !entry.name.startsWith(".") && !SKIPPED_DIRECTORIES.has(entry.name)) {
        found.push(...(await changedUnder(path, 0, sinceMs, budget)));
      }
      continue;
    }
    if (!entry.isFile() || !RENDERABLE.has(extname(entry.name).toLowerCase())) continue;
    const info = await stat(path).catch(() => null);
    if (info !== null && info.mtimeMs >= sinceMs) found.push({ path, mtimeMs: info.mtimeMs });
  }
  return found;
}

async function isRenderableFile(path: string): Promise<boolean> {
  if (!RENDERABLE.has(extname(path).toLowerCase())) return false;
  const info = await stat(path).catch(() => null);
  return info !== null && info.isFile();
}

/** pi-durable's bash spills past these, keeping the tail; Pi's card states them in its notice. */
const SPILL_AFTER_LINES = 2_000;

const SPILL_AFTER_BYTES = 50 * 1024;

function newlines(bytes: Buffer): number {
  let count = 0;
  for (let at = bytes.indexOf(0x0a); at !== -1; at = bytes.indexOf(0x0a, at + 1)) count += 1;
  return count;
}

function lineCount(text: string): number {
  const body = Buffer.from(text);
  return body.length > 0 && body[body.length - 1] !== 0x0a ? newlines(body) + 1 : newlines(body);
}

/** Lines of the spill file, read a chunk at a time: the whole output can be larger than the gateway may hold. */
async function spillLineCount(path: string): Promise<number> {
  let lines = 0;
  let last = 0x0a;
  for await (const chunk of createReadStream(path) as AsyncIterable<Buffer>) {
    lines += newlines(chunk);
    last = chunk[chunk.length - 1] ?? last;
  }
  return last === 0x0a ? lines : lines + 1;
}

/**
 * The record Pi's bash card reads, from the retained tail and the spill file that holds the whole output. The tail
 * starts at a line boundary as far as the card can tell, so the partial-line flags stay false.
 */
async function truncationOf(retained: string, fullOutputPath: string): Promise<TruncationRecord> {
  const totalBytes = (await stat(fullOutputPath)).size;
  const totalLines = await spillLineCount(fullOutputPath);
  return {
    truncated: true,
    truncatedBy: totalLines > SPILL_AFTER_LINES ? "lines" : "bytes",
    totalLines,
    totalBytes,
    outputLines: lineCount(retained),
    outputBytes: Buffer.byteLength(retained),
    lastLinePartial: false,
    firstLineExceedsLimit: false,
    maxLines: SPILL_AFTER_LINES,
    maxBytes: SPILL_AFTER_BYTES,
  };
}

/** pi-durable's `bash` with an explicit environment; its settle step builds Pi's card details from `structuredOutput` and renders the files the command created, changed or named. */
export function bashTool(options: BashToolOptions): AgentTool<"bash"> {
  const started = new Map<string, { readonly cwd: string; readonly atMs: number }>();
  /** Each scanned file's modification time when it was last attached: commands running together all see one write, and only the first to finish attaches it. */
  const attached = new Map<string, number>();
  const bash = createBashTool({
    prepare: (execution, api) => {
      execution.inheritEnv = false;
      execution.env = bashEnvironment(options.install, options.gatewayEnv);
      started.set(api.callId, { cwd: execution.cwd, atMs: Date.now() });
    },
  });
  return agentTool("bash", bash, {
    replay: "unsafe",
    background: { details: { truncation: null, fullOutputPath: null, exitCode: null, render: null } },
    wholeResult: null,
    fullTextAt: (result) => shellOutput(result)?.fullOutputPath ?? null,
    settle: async (result, call) => {
      const out = shellOutput(result);
      const run = started.get(call.id) ?? null;
      started.delete(call.id);
      const candidates = new Set<string>();
      if (out !== null && run !== null) {
        for (const path of namedIn(out.output, run.cwd)) if (await isRenderableFile(path)) candidates.add(path);
        const budget = { left: SCAN_BUDGET };
        const changed = [
          ...(await changedUnder(run.cwd, 1, run.atMs - 1_000, budget)),
          ...(await changedUnder(tmpdir(), 0, run.atMs - 1_000, budget)),
        ];
        for (const file of changed) {
          if (attached.get(file.path) === file.mtimeMs) continue;
          attached.set(file.path, file.mtimeMs);
          candidates.add(file.path);
        }
      }
      const command = typeof call.arguments["command"] === "string" ? call.arguments["command"] : "";
      const rendering =
        candidates.size === 0
          ? null
          : await options.renderer.render(
              [...candidates].map((path) => ({ path, sha256: null })),
              `bash: ${command.split("\n")[0] ?? ""}`,
            );
      return {
        ...result,
        output: [
          ...(result.output ?? []),
          ...(rendering === null ? [] : [{ type: "text" as const, text: rendering.text }, ...rendering.images]),
        ],
        details: {
          truncation:
            out !== null && out.truncated && out.fullOutputPath !== undefined
              ? await truncationOf(out.output, out.fullOutputPath)
              : null,
          fullOutputPath: out?.fullOutputPath ?? null,
          exitCode: out?.exitCode ?? null,
          render: rendering?.details ?? null,
        },
      };
    },
  });
}
