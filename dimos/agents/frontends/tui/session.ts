import type { Ref } from "@earendil-works/chord";
import type { Client } from "../../sdk/main.ts";
import type { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import type { LifecycleContract } from "../../shared/protocol/lifecycle/contract.ts";
import type { Feedback } from "./feedback.ts";

export type Ui = Client & {
  readonly gateway: Ref<typeof GatewayContract>;
  readonly lifecycle: Ref<typeof LifecycleContract>;
  readonly actions: Feedback;
};
