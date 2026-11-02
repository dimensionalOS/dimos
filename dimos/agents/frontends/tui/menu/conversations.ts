import type { MenuTab } from "#dimos/agents/frontends/tui/menu/base.ts";
import { switcherLine, switcherRows } from "#dimos/agents/shared/view-models/gateway/switcher.ts";

/** Tab 2: every conversation as the switcher orders it; Enter shows it in the chat. */
export const conversationsTab: MenuTab = {
  title: "Conversations",
  rows: (context) => {
    const gateway = context.ui.gateway;
    return switcherRows(gateway.conversations.value, context.current, context.now).map((row) => ({
      text: switcherLine(row),
      level: row.level,
      keys: [],
      open: () => {
        context.attach(row.id);
        return null;
      },
    }));
  },
};
