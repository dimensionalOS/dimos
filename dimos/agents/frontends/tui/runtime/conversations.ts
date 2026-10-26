import type { RuntimeTab } from "./base.ts";
import { switcherLine, switcherRows } from "../../../shared/view-models/gateway/switcher.ts";

/** Tab 2: every conversation as the switcher orders it; Enter shows it in the chat. */
export const conversationsTab: RuntimeTab = {
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
