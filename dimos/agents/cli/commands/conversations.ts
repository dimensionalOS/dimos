import { connect } from "../../sdk/main.ts";
import { unixSocket } from "../../sdk/node.ts";
import { refusalOfFailure } from "../../sdk/failures.ts";
import { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import type { Ref } from "@earendil-works/chord";
import type { EpochMs } from "../../shared/protocol/time.ts";
import { switcherLine, switcherRows } from "../../shared/view-models/gateway/switcher.ts";
import { refusalText } from "../../shared/view-models/outcome.ts";
import type { Command } from "./base.ts";

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
