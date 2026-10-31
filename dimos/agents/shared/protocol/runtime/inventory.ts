import type { Static } from "typebox";
import { AgentModuleRecordSchema } from "./module.ts";
import { Type } from "typebox";
import { nullable } from "../schema.ts";

const RpcSchema = Type.Object({
  name: Type.String(),
  signature: Type.String(),
  /** First line of the docstring in the whole-system result, the whole docstring in an RPC result. */
  doc: nullable(Type.String()),
});

export const InventorySchema = Type.Object({
  environment: Type.Object({
    python: Type.String(),
    dimos_root: Type.String(),
    dimos_version: nullable(Type.String()),
    dimos_commit: nullable(Type.String()),
    transport: Type.String(),
    robot_ip: nullable(Type.String()),
    robot_ips: nullable(Type.String()),
    zenoh_connect: nullable(Type.String()),
    zenoh_scouting: Type.Boolean(),
    /** Path of `docs/usage/python-api.md`. */
    python_api_guide: nullable(Type.String()),
    relay: nullable(Type.Object({ url: Type.String(), robot: Type.String() })),
  }),
  runtime: Type.Object({
    modules: Type.Array(
      Type.Object({
        instance: Type.String(),
        class: nullable(Type.String()),
        path: nullable(Type.String()),
        doc: nullable(Type.String()),
        rpcs: Type.Array(RpcSchema),
      }),
    ),
    streams: Type.Array(
      Type.Object({
        module: Type.String(),
        name: Type.String(),
        direction: nullable(Type.Union([Type.Literal("in"), Type.Literal("out"), Type.Literal("inout")])),
        type: Type.String(),
        channel: nullable(Type.String()),
      }),
    ),
  }),
  agent_modules: Type.Array(AgentModuleRecordSchema),
});

/** The whole running system returned by `inventory`. */
export type Inventory = Static<typeof InventorySchema>;
