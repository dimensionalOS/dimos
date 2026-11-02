# Command line

`dimos agent` is the command line for the agent. Python's `dimos agent` in [agent.py](/dimos/cli/commands/agent.py)
builds this package when it changed and runs `node dist/cli.js`, which [build.mjs](/dimos/agents/build.mjs) bundles from
[main.ts](/dimos/agents/cli/main.ts). `main.ts` parses the arguments and runs one command from [commands/](/dimos/agents/cli/commands/). You come here to
add a command.

Source: [dimos/agents/cli/](/dimos/agents/cli/). TypeScript examples use `#dimos/agents/...` package subpaths.
Save temporary example files there and run them from `dimos/agents` with `node --import=./test/web-sdk.mjs cli/demo_example.ts`.
Remove the temporary file with `rm cli/demo_example.ts` before running `npm run typecheck` or `npm test`.
Those checks treat saved examples as package code and may reject their imports.

## Built on the SDK

A command is a client of the gateway. It connects with `connect(unixSocket({ path, label }))` from the
[SDK](/docs/usage/agent/sdk.md), holds the gateway's objects with `client.host.get`, and reads their cells. `connect` redials
until a gateway answers, so a command first checks for one with `gatewayState` or starts one with `startGateway`.

## Commands

| Command                                               | What it does                                                                                                                |
| ----------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `[--conversation ID] [--view] [--cwd DIR]`            | Opens the [terminal](/docs/development/agent/tui.md) on a new conversation, or on `--conversation`. `--view` opens it read-only |
| `run [--conversation ID] [--cwd DIR] [--json] PROMPT` | Sends `PROMPT` to a new conversation, or to `--conversation`, and streams the answer, or JSON lines with `--json`           |
| `status [--json]`                                     | Prints the gateway, its dimOS link, modules, jobs and streams                                                               |
| `conversations`                                       | Lists the conversations, newest first                                                                                       |
| `gateway [--daemon]`                                  | Runs the gateway in the foreground, or in the background with `--daemon`                                                    |
| `stop`                                                | Stops the gateway                                                                                                           |
| `version`                                             | Prints this build                                                                                                           |

`dimos agent` with no command starts the gateway in the background when none runs, then opens the terminal through
[tui.ts](/dimos/agents/cli/commands/tui.ts), which only draws it. `run` and `conversations` also start the gateway when none runs. Python
runs `load` and `service` itself and hands every other command line to Node unparsed.

## Quickstart

A command takes a `CommandContext` and returns its exit code. This one prints the gateway's pid without connecting, the
way [status.ts](/dimos/agents/cli/commands/status.ts) first checks for a gateway. Add it to `commands` and `USAGE` in `main.ts`.

```ts
import type { Command } from "#dimos/agents/cli/commands/base.ts";
import { gatewayState } from "#dimos/agents/cli/process.ts";

export const runPid: Command = async ({ paths }) => {
    const state = await gatewayState(paths, 500);
    if (state.kind !== "running") return 3;
    process.stdout.write(`${state.process.pid}\n`);
    return 0;
};
```

A command that reads the gateway's state connects through the SDK, as
[conversations.ts](/dimos/agents/cli/commands/conversations.ts) does.

```ts
import { GatewayContract } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { connect } from "#dimos/agents/sdk/main.ts";
import { describeLink } from "#dimos/agents/shared/protocol/gateway/link.ts";
import { unixSocket } from "#dimos/agents/sdk/node.ts";

export const runLink: Command = async ({ paths, startGateway }) => {
    await startGateway();
    const client = await connect(unixSocket({ path: paths.socket, label: "link" }));
    const gateway = await client.host.get(GatewayContract, {});
    process.stdout.write(`${describeLink(gateway.link.value)}\n`);
    gateway.release();
    await client.close();
    return 0;
};
```

## API

[process.ts](/dimos/agents/cli/process.ts) finds, starts and stops the gateway from outside it:

- `gatewayState(paths, timeoutMs)` returns `running`, `other_build`, `stopped`, `starting`, `stale_socket` or
  `not_answering`. `starting` means `ensureGateway` has spawned a gateway that has not bound the socket yet, so
  `dimos agent` refuses to reinstall packages while it loads them.
- `ensureGateway(paths, command, env)` spawns the gateway detached when nothing answers and waits until it serves its
  Lifecycle object. It throws when the gateway exits first, does not serve within 60 s, or is another build.
  `CommandContext.startGateway` calls it.
- `stopGateway(paths)` stops the gateway through `LifecycleContract`, which stays at version 1, so it also stops a
  gateway of another build.

[commands/base.ts](/dimos/agents/cli/commands/base.ts) declares `CommandContext`: `paths` from `agentPaths(env)`, `env`, the parsed
`--conversation`, `--cwd`, `--view`, `--json` and `--daemon`, the remaining positionals as `args`, `bundledSkillsDir`, and
`startGateway`. A new flag needs an entry in the `parseArgs` options in `main.ts` and a field here.
[boundaries.test.ts](/dimos/agents/test/boundaries.test.ts) fails when a file in `commands/` does not import `base.ts`.

### What Python does first

Python's `dimos agent` checks that `node` is version 24 or 26. Except for `gateway`, `stop` and `status`, it runs
`npm ci` when `package.json` or `package-lock.json` changed and `npm run build` when a file the last build read changed.
It drops the variables `dimos` loaded from `.env` that are not `GlobalConfig` settings, so API keys there stay out of
the gateway, and adds six that say where dimOS is. Then it replaces itself with `node dist/cli.js`.

| Variable          | Value                                                                           |
| ----------------- | ------------------------------------------------------------------------------- |
| `DIMOS_ROOT`      | The dimOS checkout                                                              |
| `DIMOS_PYTHON`    | The venv's Python                                                               |
| `DIMOS_CLI`       | The venv's `dimos` script, which the gateway runs as `dimos agent load`         |
| `DIMOS_MCP_URL`   | McpServer's URL, from `MCPSERVER__HOST` and `MCPSERVER__PORT` or their defaults |
| `DIMOS_RUNS_DIR`  | The run registry, `~/.local/state/dimos/runs/`                                  |
| `DIMOS_STATE_DIR` | dimOS's state directory, `~/.local/state/dimos/`                                |

`gateway` reads them with `parseDimosInstall` from [gateway/config/environment.ts](/dimos/agents/gateway/config/environment.ts),
and a gateway that `startGateway` spawns inherits them.
