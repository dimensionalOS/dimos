import type { Ref } from "@earendil-works/chord";
import type { Client } from "#dimos/agents/sdk/main.ts";
import type { GatewayContract } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import type { LifecycleContract } from "#dimos/agents/shared/protocol/lifecycle/contract.ts";
import type { Feedback } from "#dimos/agents/frontends/tui/feedback.ts";

export type Ui = Client & {
  readonly gateway: Ref<typeof GatewayContract>;
  readonly lifecycle: Ref<typeof LifecycleContract>;
  readonly actions: Feedback;
};
