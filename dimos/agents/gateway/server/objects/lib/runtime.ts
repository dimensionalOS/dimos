import type { StreamRef } from "../../../../shared/protocol/streams.ts";
import type { DimosConnection } from "../../../../harness/dimos/connection.ts";
import type { Inventory } from "../../../../shared/protocol/runtime/inventory.ts";
import type { InventoryView } from "../../../../shared/protocol/inventory/contract.ts";
import type { ModuleInstance, ModuleRow } from "../../../../shared/protocol/runtime/module.ts";
import type { RuntimeClient } from "../../../../harness/dimos/client.ts";
import type { Cell, Ctx, Readable, Self } from "@earendil-works/chord";
import type { LinkStatus } from "../../../../shared/protocol/gateway/link.ts";
import type { Principal } from "../../../auth/principal.ts";
import { carried } from "../../../../shared/protocol/schema.ts";
import { setInterval } from "node:timers/promises";
import { DimosUnavailable } from "../../../../harness/dimos/connection.ts";
import { epochMs } from "../../../../shared/protocol/time.ts";
import { readModuleRecords } from "../../../../harness/dimos/disk.ts";

/** What HarnessRuntime's inventory says, parsed once into the rows the UI shows. */
type RuntimeView = {
  readonly modules: readonly ModuleRow[];
  readonly streams: readonly StreamRef[];
  readonly inventory: InventoryView;
};

type RuntimeOptions = {
  readonly connection: DimosConnection;
  readonly runtime: RuntimeClient;
  /** `$STATE/dimos`, for module records while the link is down. */
  readonly stateDir: string;
  /** The backstop while the link is up. */
  readonly pollMs: number;
  readonly report: (error: unknown) => void;
};

/** The runtime module an agent module runs as, by class name. */
function instancesOf(inventory: Inventory): ReadonlyMap<string, ModuleInstance> {
  const instances = new Map<string, ModuleInstance>();
  for (const module of inventory.runtime.modules)
    if (module.class !== null && !instances.has(module.class))
      instances.set(module.class, module.instance as ModuleInstance);
  return instances;
}

function viewOf(inventory: Inventory, streams: readonly StreamRef[]): RuntimeView {
  const instances = instancesOf(inventory);
  const modules = inventory.agent_modules.map((record): ModuleRow => {
    const instance = instances.get(record.class);
    return {
      name: record.name,
      class: record.class,
      owner: record.owner,
      path: record.path,
      lastError: record.last_error,
      state: record.state,
      revision: record.revision,
      instance: instance === undefined ? null : instance,
    };
  });
  return {
    modules,
    streams,
    inventory: { inventory, at: epochMs(Date.now()), current: true },
  };
}

/** Inventory reads and polling live with the GatewayStore that owns this cell. */
export function runtimeView(
  self: Self<Principal>,
  link: Readable<LinkStatus>,
  options: RuntimeOptions,
): Cell<RuntimeView> {
  const view = self.state("runtime", carried<RuntimeView>(), {
    modules: [],
    streams: [],
    inventory: null,
  });
  let reading: Promise<void> | null = null;
  let again = false;
  const fromDisk = async (ctx: Ctx<Principal>): Promise<void> => {
    const records = await readModuleRecords(options.stateDir, options.report);
    if (ctx.signal.aborted || options.connection.status.value.state === "up") return;
    options.runtime.disconnectRelay();
    const last = view.value.inventory;
    view.replace(ctx, {
      modules: records.map(
        (record): ModuleRow => ({
          name: record.name,
          class: record.className,
          owner: record.owner,
          path: record.path,
          lastError: record.lastError,
          state: "unknown",
          revision: null,
          instance: null,
        }),
      ),
      streams: [],
      inventory: last === null ? null : { ...last, current: false },
    });
  };
  const read = async (ctx: Ctx<Principal>): Promise<void> => {
    try {
      if (options.connection.status.value.state !== "up") return await fromDisk(ctx);
      const inventory = await options.runtime.inventory(ctx.signal);
      if (!ctx.signal.aborted && options.connection.status.value.state === "up") {
        view.replace(ctx, viewOf(inventory, options.runtime.streams.value));
        if (inventory.environment.relay === null) options.runtime.disconnectRelay();
        else void options.runtime.relay(inventory).catch(options.report);
      }
    } catch (error) {
      if (ctx.signal.aborted) return;
      if (error instanceof DimosUnavailable) await fromDisk(ctx);
      else options.report(error);
    }
  };
  const refresh = (ctx: Ctx<Principal>): Promise<void> => {
    again = true;
    if (reading !== null) return reading;
    reading = (async () => {
      do {
        again = false;
        await read(ctx);
      } while (again && !ctx.signal.aborted);
    })().finally(() => {
      reading = null;
    });
    return reading;
  };
  self.on("runtimeLink", link, (_link, ctx) => refresh(ctx));
  self.on("runtimeStreams", options.runtime.streams, (streams, ctx) => {
    view.replace(ctx, { ...view.value, streams });
  });
  self.on(
    "runtimeStatus",
    { subscribe: (listener: () => void) => options.connection.onHarnessStatus(listener) },
    (_event, ctx) => refresh(ctx),
  );
  self.task(async (ctx) => {
    try {
      for await (const _ of setInterval(options.pollMs, undefined, { signal: ctx.signal, ref: false })) {
        if (options.connection.status.value.state === "up") await refresh(ctx);
      }
    } catch (error) {
      if (!ctx.signal.aborted) throw error;
    }
  });
  return view;
}
