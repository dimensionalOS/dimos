import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { parseArgs } from "node:util";
import type { ConversationId } from "@earendil-works/pi-durable";
import { agentPaths } from "#dimos/agents/gateway/config/paths.ts";
import type { Command, CommandContext } from "#dimos/agents/cli/commands/base.ts";
import { runConversations } from "#dimos/agents/cli/commands/conversations.ts";
import { runGatewayCommand } from "#dimos/agents/cli/commands/gateway.ts";
import { printRun } from "#dimos/agents/cli/commands/run.ts";
import { runStatus } from "#dimos/agents/cli/commands/status.ts";
import { runStop } from "#dimos/agents/cli/commands/stop.ts";
import { runTui } from "#dimos/agents/cli/commands/tui.ts";
import { runVersion } from "#dimos/agents/cli/commands/version.ts";
import { ensureGateway } from "#dimos/agents/cli/process.ts";

const USAGE = `dimos agent [--conversation ID] [--view] [--cwd DIR]              open a terminal (a new conversation without --conversation)
dimos agent run [--conversation ID] [--cwd DIR] [--json] PROMPT   submit one input and print the answer, as JSON lines with --json
dimos agent conversations                                         list conversations, newest first
dimos agent gateway [--daemon]                                    run the gateway, in the background with --daemon
dimos agent status [--json]                                       the gateway, DimOS, conversations, modules, jobs, streams
dimos agent stop | version`;

const commands: ReadonlyMap<string, Command> = new Map([
  ["gateway", runGatewayCommand],
  ["stop", runStop],
  ["status", runStatus],
  ["version", runVersion],
  ["conversations", runConversations],
  ["run", printRun],
]);

function conversationId(value: string | undefined): ConversationId | null {
  if (value === undefined) return null;
  const id = Number(value);
  if (!Number.isSafeInteger(id) || id < 1) throw new Error(`--conversation takes a conversation number, not ${value}`);
  return id as ConversationId;
}

async function main(argv: readonly string[], env: NodeJS.ProcessEnv): Promise<number> {
  const major = Number(process.versions.node.split(".")[0]);
  if (major !== 24 && major !== 26) {
    process.stderr.write(`dimos agent needs Node 24 or 26; this is ${process.version}.\n`);
    return 1;
  }
  const { values, positionals } = parseArgs({
    args: [...argv],
    allowPositionals: true,
    options: {
      conversation: { type: "string" },
      view: { type: "boolean", default: false },
      json: { type: "boolean", default: false },
      daemon: { type: "boolean", default: false },
      cwd: { type: "string" },
      help: { type: "boolean", default: false },
    },
  });
  const [command, ...args] = positionals;
  if (values.help || command === "help") {
    process.stdout.write(`${USAGE}\n`);
    return 0;
  }
  const paths = agentPaths(env);
  const context: CommandContext = {
    paths,
    env,
    cwd: values.cwd === undefined ? null : resolve(values.cwd),
    conversation: conversationId(values.conversation),
    view: values.view,
    json: values.json,
    daemon: values.daemon,
    args,
    // Both cli/main.ts and dist/cli.js are one directory below the package root.
    bundledSkillsDir: join(dirname(fileURLToPath(import.meta.url)), "..", "harness", "skills"),
    startGateway: () =>
      ensureGateway(
        paths,
        [process.execPath, ...process.execArgv, resolve(process.argv[1] ?? "cli/main.ts"), "gateway"],
        env,
      ),
  };
  if (command === undefined) {
    if (context.view && context.conversation === null) {
      process.stderr.write("--view needs --conversation ID.\n");
      return 2;
    }
    await context.startGateway();
    return runTui(context);
  }
  const run = commands.get(command);
  if (run === undefined) {
    process.stderr.write(`Unknown command: ${command}\n${USAGE}\n`);
    return 2;
  }
  return run(context);
}

/** A piped stdout is written asynchronously; exiting before it drains cuts a long `status --json` or a printed answer. */
function drained(stream: NodeJS.WriteStream): Promise<void> {
  return new Promise((resolve) => stream.write("", () => resolve()));
}

main(process.argv.slice(2), process.env).then(
  async (code) => {
    await Promise.all([drained(process.stdout), drained(process.stderr)]);
    process.exit(code);
  },
  async (error: unknown) => {
    process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
    await drained(process.stderr);
    process.exit(1);
  },
);
