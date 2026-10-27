import type { ModuleInstance } from "../../../shared/protocol/runtime/module.ts";
import type { MenuRow, MenuTab } from "./base.ts";
import type { StreamRef } from "../../../shared/protocol/streams.ts";
import { plain } from "./lib/rows.ts";

/** Tab 5: the relay's readable channels, annotated with module inventory when available. */
export const streamsTab: MenuTab = {
  title: "Streams",
  rows: ({ ui }) => {
    const groups = new Map<ModuleInstance | null, StreamRef[]>();
    for (const stream of ui.gateway.streams.value) {
      const instance = stream.source?.module ?? null;
      groups.set(instance, [...(groups.get(instance) ?? []), stream]);
    }
    const agentModules = ui.gateway.modules.value;
    const rows: MenuRow[] = [];
    for (const [instance, streams] of groups) {
      const agent = instance === null ? undefined : agentModules.find((module) => module.instance === instance);
      const owner =
        agent === undefined ? "" : ` (agent module ${agent.name}${agent.owner === null ? "" : `, #${agent.owner}`})`;
      rows.push(plain(`${instance ?? "Relay"}${owner}`));
      for (const stream of streams) rows.push(plain(`  ${stream.channel}  ${stream.encoding}`));
    }
    return rows;
  },
};
