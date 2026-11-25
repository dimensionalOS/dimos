import { owner } from "#dimos/agents/sdk/failures.ts";
import type { ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { MenuTab } from "#dimos/agents/frontends/tui/menu/base.ts";

/** "follower  #2  r12 running  ✖ ImportError: …" */
function moduleText(module: ModuleRow): string {
  return (
    `${module.name}  ${module.owner === null ? "none" : `#${module.owner}`}  ` +
    `${module.revision === null ? "" : `r${module.revision} `}${module.state}` +
    (module.lastError === null ? "" : `  ✖ ${module.lastError.split("\n")[0] ?? ""}`)
  );
}

export const modulesTab: MenuTab = {
  title: "Agent modules",
  rows: (context) => {
    const { ui } = context;
    return ui.gateway.modules.value.map((module) => ({
      text: moduleText(module),
      searchText: module.name,
      level: module.lastError === null ? "ok" : "warn",
      actions: [
        {
          label: "restart",
          run: async () => {
            await ui.actions.observe(`restart ${module.name}`, owner(ui.gateway).restartModule({ name: module.name }));
          },
        },
        {
          label: "unload",
          run: async () => {
            await ui.actions.observe(`unload ${module.name}`, owner(ui.gateway).unloadModule({ name: module.name }));
          },
        },
        {
          label: "delete",
          run: async () => {
            await ui.actions.observe(`delete ${module.name}`, owner(ui.gateway).deleteModule({ name: module.name }));
          },
        },
      ],
      open: null,
    }));
  },
};
