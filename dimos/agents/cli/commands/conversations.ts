import { connect } from "#dimos/agents/sdk/main.ts";
import { unixSocket } from "#dimos/agents/sdk/node.ts";
import { refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
import { GatewayContract } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import type { Ref } from "@earendil-works/chord";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import { switcherLine, switcherRows } from "#dimos/agents/shared/view-models/gateway/switcher.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";
import type { Command } from "#dimos/agents/cli/commands/base.ts";

/** `dimos agent conversations`: the switcher's rows, newest roots first with their forks under them. */
export const runConversations: Command = async ({ paths, startGateway }) => {
  await startGateway();
  try {
    const client = await connect(unixSocket({ path: paths.socket, label: "conversations" }));
    let gateway: Ref<typeof GatewayContract> | undefined;
    try {
      gateway = await Promise.race([client.host.get(GatewayContract, {}), client.ended]);
      const rows = switcherRows(gateway.conversations.value, null, Date.now() as EpochMs);
      for (const row of rows) process.stdout.write(`${switcherLine(row)}\n`);
      return 0;
    } finally {
      gateway?.release();
      await client.close();
    }
  } catch (error) {
    process.stderr.write(`${refusalText(refusalOfFailure(error, Date.now() as EpochMs))}\n`);
    return 1;
  }
};
