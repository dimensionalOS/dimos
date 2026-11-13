import type { Inventory } from "#dimos/agents/shared/protocol/runtime/inventory.ts";
import type { MenuDetail, MenuRow, MenuTab, TabContext } from "#dimos/agents/frontends/tui/menu/base.ts";
import { plain } from "#dimos/agents/frontends/tui/menu/lib/rows.ts";

/** A nullable inventory field as shown: "-" when DimOS did not report it. */
function or(value: string | null): string {
  return value ?? "-";
}

/** " · doc" for a non-empty doc. */
function suffix(doc: string | null): string {
  return doc === null || doc === "" ? "" : ` · ${doc}`;
}

/** The held inventory, or null before its first value. */
function inventoryOf(context: TabContext): Inventory | null {
  const view = context.inventory?.inventory.value;
  return view === undefined || view === null ? null : view.inventory;
}

function withInventory(
  rows: (inventory: Inventory) => readonly MenuRow[],
): (context: TabContext) => readonly MenuRow[] {
  return (context) => {
    const inventory = inventoryOf(context);
    return inventory === null ? [plain("Waiting for the inventory…")] : rows(inventory);
  };
}

function moduleDetail(inventory: Inventory, instance: string): MenuDetail {
  const module = inventory.runtime.modules.find((candidate) => candidate.instance === instance);
  const streams = inventory.runtime.streams.filter((stream) => stream.module === instance);
  return {
    title: instance,
    lines: () => [
      "Streams",
      ...streams.map((stream) => `  ${stream.name} ${or(stream.direction)} ${stream.type} ${or(stream.channel)}`),
      "RPCs",
      ...(module?.rpcs ?? []).map((rpc) => `  ${rpc.signature}${suffix(rpc.doc)}`),
    ],
    close: async () => {},
  };
}

/** Tab 6: the DimOS checkout, Python, transport and system size. */
export const environmentTab: MenuTab = {
  title: "Env",
  rows: withInventory((inventory) => {
    const env = inventory.environment;
    return [
      plain(`DimOS          ${or(env.dimos_version)} @ ${or(env.dimos_commit)}`),
      plain(`Checkout       ${env.dimos_root}`),
      plain(`Python         ${env.python}`),
      plain(`Transport      ${env.transport}${env.robot_ip === null ? "" : ` · robot_ip ${env.robot_ip}`}`),
      plain(`Zenoh          ${or(env.zenoh_connect)}${env.zenoh_scouting ? " · scouting" : ""}`),
      plain(`Relay          ${env.relay === null ? "-" : `${env.relay.url} · ${env.relay.robot}`}`),
      plain(`API guide      ${or(env.python_api_guide)}`),
      plain(
        `System         ${inventory.runtime.modules.length} modules · ${inventory.runtime.streams.length} streams · ${inventory.agent_modules.length} agent modules`,
      ),
    ];
  }),
};

/** Tab 7: every module instance of the running system; Enter lists its streams and RPCs. */
export const inventoryModulesTab: MenuTab = {
  title: "Modules",
  rows: withInventory((inventory) =>
    inventory.runtime.modules.map((module) => {
      const streams = inventory.runtime.streams.filter((stream) => stream.module === module.instance);
      return {
        text: `${module.instance} [${or(module.class)}] · ${streams.length} streams · ${module.rpcs.length} rpcs${suffix(module.doc)}`,
        level: "ok",
        keys: [],
        open: () => moduleDetail(inventory, module.instance),
      };
    }),
  ),
};

/** Tab 8: every RPC with its signature and the first line of its docstring. */
export const rpcsTab: MenuTab = {
  title: "RPCs",
  rows: withInventory((inventory) =>
    inventory.runtime.modules.flatMap((module) =>
      module.rpcs.map((rpc) => plain(`${module.instance}.${rpc.signature}${suffix(rpc.doc)}`)),
    ),
  ),
};

/** Tab 9: every stream with its direction, message type and channel. */
export const inventoryStreamsTab: MenuTab = {
  title: "Stream types",
  rows: withInventory((inventory) =>
    inventory.runtime.streams.map((stream) =>
      plain(`${stream.module}.${stream.name} ${or(stream.direction)} ${stream.type} ${or(stream.channel)}`),
    ),
  ),
};
