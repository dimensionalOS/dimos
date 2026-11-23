import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import type { ChildProcessWithoutNullStreams } from "node:child_process";
import { Type } from "typebox";
import { spawn } from "node:child_process";
import { access } from "node:fs/promises";
import { dirname, join, resolve } from "node:path";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";

const Parameters = Type.Object({
  op: Type.Literal("inspect"),
  path: Type.Optional(
    Type.String({ description: "Sqlite recording. Defaults to recording.db then memory.db in the workspace." }),
  ),
  t: Type.Union([
    Type.Number({ description: "Seconds from the first color_image frame." }),
    Type.Array(Type.Number(), {
      minItems: 1,
      maxItems: 4,
      description:
        "Up to four relative seconds. Returns one labeled strip (2–3) or 2x2 grid (4) for motion/direction compare.",
    }),
  ]),
  ask: Type.Optional(
    Type.String({
      description:
        "Questions to answer from this frame or montage. Cover both competing answers: some direct, some that would refute the pick or corroborate it indirectly.",
    }),
  ),
});

const argumentsDecoder = wireDecoder("memory arguments", Parameters);

export type MemoryHit = {
  readonly t: number;
  readonly id: number;
  readonly score: number | null;
  readonly path: string | null;
  readonly caption?: string | null;
};

export type MemoryResult = {
  readonly ok: boolean;
  readonly error?: string;
  readonly op?: string;
  readonly origin_ts?: number;
  readonly stream?: string;
  readonly hits?: readonly MemoryHit[];
  readonly montage?: string | null;
};

function hitsWithoutCaption(result: MemoryResult): MemoryResult {
  return {
    ...result,
    hits: (result.hits ?? []).map((hit) => {
      const { caption: _caption, ...rest } = hit;
      return rest;
    }),
  };
}

export type MemoryRequest = {
  readonly op: string;
  readonly path: string;
  readonly out_dir: string;
  readonly t?: number | readonly number[];
};

const DESCRIPTION = [
  "Open color frame(s) from a DimOS recording (sqlite memory store). t is seconds from the first color_image frame, not Unix time.",
  "inspect: nearest color frame(s), never the 1 Hz caption. t may be one number or a list of up to 4 times; 2–3 times become one horizontal strip with a t= label on each cell, 4 times a 2x2 grid. Use a multi-t inspect for motion and left/right (compare positions across the strip), not a dense sweep. Pass ask with several questions covering both competing answers: some direct, some that would refute the pick or corroborate it indirectly.",
  "Index first, in several rounds: search captions with rg if it is on PATH, otherwise grep -E, then head/tail, on captions.jsonl or captions/YYYY-MM-DD.jsonl (see captions/manifest.jsonl). Grep tracks.jsonl the same way. Whole recording, then synonyms and the question's time constraint. Later hits are other candidates. A gap of several seconds is a new episode. If choices are timestamps, inspect those t values. Unique-count from distinct track ids; at most one inspect per id. Captions and tracks are a grep index only.",
  "Inspect only when the index cannot answer (color, motion, barriers, door numbers, open/closed, spatial referents). Inspect the shortlisted t in that window, not a dense sweep and not weakly related earlier/later segments.",
  "Before the final answer, pause: restate the question and competing choices, name the index hits and any inspected t values, and check unique vs repeated views, earlier co-occurrence vs a later frame, and duration endpoints.",
  "Do not open the sqlite file from bash.",
].join(" ");

async function existing(path: string): Promise<string | null> {
  try {
    await access(path);
    return path;
  } catch {
    return null;
  }
}

export async function resolveRecording(workspace: string, path: string | undefined): Promise<string> {
  if (path !== undefined) {
    const resolved = resolve(workspace, path);
    const found = await existing(resolved);
    if (found === null) throw new Error(`recording not found: ${resolved}`);
    return found;
  }
  for (const name of ["recording.db", "memory.db"]) {
    const found = await existing(join(workspace, name));
    if (found !== null) return found;
  }
  throw new Error(`no recording.db or memory.db in ${workspace}; pass path`);
}

class QueryWorker {
  private child: ChildProcessWithoutNullStreams | null = null;
  private buffer = "";
  private wait: { resolve: (value: MemoryResult) => void; reject: (error: unknown) => void } | null = null;
  private queue: Promise<void> = Promise.resolve();
  private readonly python: string;
  private readonly root: string;

  constructor(python: string, root: string) {
    this.python = python;
    this.root = root;
  }

  request(body: MemoryRequest, signal: AbortSignal | undefined): Promise<MemoryResult> {
    const run = (): Promise<MemoryResult> =>
      new Promise((resolve, reject) => {
        if (this.wait !== null) {
          reject(new Error("memory worker is already waiting on a request"));
          return;
        }
        const child = this.ensure();
        const onAbort = () => {
          this.fail(signal?.reason ?? new Error("aborted"));
        };
        signal?.addEventListener("abort", onAbort, { once: true });
        this.wait = {
          resolve: (value) => {
            signal?.removeEventListener("abort", onAbort);
            resolve(value);
          },
          reject: (error) => {
            signal?.removeEventListener("abort", onAbort);
            reject(error);
          },
        };
        child.stdin.write(`${JSON.stringify(body)}\n`);
      });
    const next = this.queue.then(run, run);
    this.queue = next.then(
      () => undefined,
      () => undefined,
    );
    return next;
  }

  close(): void {
    this.fail(new Error("memory worker closed"));
    this.child?.kill();
    this.child = null;
  }

  private ensure(): ChildProcessWithoutNullStreams {
    if (this.child !== null) return this.child;
    const child = spawn(this.python, ["-m", "dimos.memory.query"], {
      cwd: this.root,
      env: { ...process.env, DIMOS_ROOT: this.root, PYTHONUNBUFFERED: "1" },
      stdio: ["pipe", "pipe", "pipe"],
    });
    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk: string) => {
      this.buffer += chunk;
      const lines = this.buffer.split("\n");
      this.buffer = lines.pop() ?? "";
      for (const line of lines) {
        if (line.trim() === "") continue;
        let parsed: MemoryResult;
        try {
          parsed = JSON.parse(line) as MemoryResult;
        } catch (error) {
          this.fail(error);
          return;
        }
        const wait = this.wait;
        this.wait = null;
        wait?.resolve(parsed);
      }
    });
    child.on("exit", (code, signal) => {
      this.child = null;
      this.fail(new Error(`memory worker exited (${code ?? signal ?? "unknown"})`));
    });
    this.child = child;
    return child;
  }

  private fail(error: unknown): void {
    const wait = this.wait;
    this.wait = null;
    wait?.reject(error);
  }
}

function attachFiles(result: MemoryResult): string[] {
  const montage = result.montage;
  if (typeof montage === "string" && montage !== "") return [montage];
  return (result.hits ?? [])
    .map((hit) => hit.path)
    .filter((file): file is string => typeof file === "string" && file !== "");
}

export function memoryTool(options: {
  readonly install: DimosInstall;
  readonly renderer: FileRenderer;
  readonly workspace: () => string;
  readonly query?: (request: MemoryRequest, signal: AbortSignal | undefined) => Promise<MemoryResult>;
}): AgentTool<"memory"> {
  const worker = options.query === undefined ? new QueryWorker(options.install.python, options.install.root) : null;
  const query = options.query ?? ((request, signal) => worker!.request(request, signal));
  return agentTool(
    "memory",
    {
      name: "memory",
      description: DESCRIPTION,
      parameters: Parameters,
      execute: async (args, _api, context) => {
        try {
          const input = argumentsDecoder.decode(args);
          const workspace = options.workspace();
          const path = await resolveRecording(workspace, input.path);
          const result = await query(
            {
              op: input.op,
              path,
              out_dir: join(dirname(path), "memory-frames"),
              t: input.t,
            },
            context.abortSignal,
          );
          if (result.ok !== true)
            return {
              output: [{ type: "text", text: result.error ?? "memory query failed" }],
              isError: true,
              details: { render: null },
            };
          const files = attachFiles(result);
          const rendered =
            files.length === 0
              ? null
              : await options.renderer.render(
                  files.map((file) => ({ path: file, sha256: null })),
                  `memory ${input.op}`,
                );
          const ask = input.ask?.trim() ?? "";
          const look =
            Array.isArray(input.t) && input.t.length > 1
              ? "Look at the labeled frames (each cell has t=…) and answer:\n"
              : "Look at the image and answer:\n";
          return {
            output: [
              { type: "text", text: JSON.stringify(hitsWithoutCaption(result)) },
              ...(ask === "" ? [] : [{ type: "text" as const, text: `${look}${ask}` }]),
              ...(rendered === null ? [] : [{ type: "text" as const, text: rendered.text }]),
            ],
            isError: false,
            details: { render: rendered?.details ?? null },
          };
        } catch (error) {
          context.abortSignal?.throwIfAborted();
          return { output: [{ type: "text", text: String(error) }], isError: true, details: { render: null } };
        }
      },
    },
    { replay: "safe", settle: null, fullTextAt: null, wholeResult: null, background: { details: { render: null } } },
  );
}
