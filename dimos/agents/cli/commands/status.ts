import type { Ref } from "@earendil-works/chord";
import { connect } from "../../sdk/main.ts";
import { unixSocket } from "../../sdk/node.ts";
import { refusalOfFailure } from "../../sdk/failures.ts";
import { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import { LifecycleContract } from "../../shared/protocol/lifecycle/contract.ts";
import type { EpochMs } from "../../shared/protocol/time.ts";
import { refusalText } from "../../shared/view-models/outcome.ts";
import { statusText } from "../../shared/view-models/status.ts";
import { gatewayState } from "../process.ts";
import type { Command } from "./base.ts";

/** What `dimos agent status --json` prints: the gateway process and every gateway member's value. */
export function statusJson(refs: { gateway: Ref<typeof GatewayContract>; lifecycle: Ref<typeof LifecycleContract> }) {
  const gateway = refs.gateway;
  return {
    kind: "running" as const,
    process: refs.lifecycle.process.value,
    gateway: {
      link: gateway.link.value,
      load: gateway.load.value,
      runs: gateway.runs.value,
      conversations: gateway.conversations.value,
      modules: gateway.modules.value,
      streams: gateway.streams.value,
      jobs: gateway.jobs.value,
      settings: gateway.settings.value,
      accounts: gateway.accounts.value,
      loginFlow: gateway.loginFlow.value,
    },
  };
}

/**
 * `dimos agent status` reports the process state, or connects as `status`, prints the text or the JSON, and closes. A gateway
 * of another build answers through Lifecycle alone, so its build and the restart hint still print, with exit 1.
 */
export const runStatus: Command = async ({ paths, json }) => {
  const state = await gatewayState(paths, 500);
  if (state.kind === "other_build") {
    process.stdout.write(
      json
        ? `${JSON.stringify({ kind: "other_build", process: state.process }, null, 2)}\n`
        : `${"gateway".padEnd(11)}✖ another build (${state.process.build.build}); restart it with \`dimos agent stop\`, or with \`systemctl --user restart dimos-agent.service\` when it runs as the service\n`,
    );
    return 1;
  }
  if (state.kind !== "running") {
    const words = {
      stopped: "not running",
      starting: "starting",
      stale_socket: `not running; a stale ${paths.socket} is left`,
      not_answering: `a process holds ${paths.socket} but does not answer`,
    }[state.kind];
    process.stdout.write(json ? `${JSON.stringify(state, null, 2)}\n` : `gateway  ${words}\n`);
    return state.kind === "stopped" ? 3 : 1;
  }
  try {
    const client = await connect(unixSocket({ path: paths.socket, label: "status" }));
    let lifecycle: Ref<typeof LifecycleContract> | undefined;
    let gateway: Ref<typeof GatewayContract> | undefined;
    try {
      lifecycle = await Promise.race([client.host.get(LifecycleContract, {}), client.ended]);
      gateway = await Promise.race([client.host.get(GatewayContract, {}), client.ended]);
      const refs = { gateway, lifecycle };
      process.stdout.write(
        json
          ? `${JSON.stringify(statusJson(refs), null, 2)}\n`
          : `${statusText(refs, Date.now() as EpochMs).join("\n")}\n`,
      );
      return 0;
    } finally {
      gateway?.release();
      lifecycle?.release();
      await client.close();
    }
  } catch (error) {
    process.stderr.write(`${refusalText(refusalOfFailure(error, Date.now() as EpochMs))}\n`);
    return 1;
  }
};
