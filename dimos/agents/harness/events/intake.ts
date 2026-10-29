import type { Context } from "@earendil-works/chord/context";
import type { DimosConnection } from "../dimos/connection.ts";
import type { EventDrafts } from "./message.ts";
import type { HarnessEvent } from "../../shared/protocol/runtime/event.ts";
import type { InputGate } from "../inputs/gate.ts";

type EventIntakeOptions = {
  readonly connection: Pick<DimosConnection, "onEvent">;
  readonly inputs: Pick<InputGate, "submit">;
  readonly drafts: Pick<EventDrafts, "forEvent">;
  readonly report: (error: unknown) => void;
};

/** Admits live notices in arrival order; InputGate deduplicates their stable IDs. */
export class EventIntake {
  private readonly options: EventIntakeOptions;
  private unsubscribe: (() => void) | null = null;
  private pending: Promise<void> = Promise.resolve();

  constructor(options: EventIntakeOptions) {
    this.options = options;
  }

  start(context: Context): void {
    if (this.unsubscribe !== null) return;
    this.unsubscribe = this.options.connection.onEvent((event) => {
      this.pending = this.pending.then(() => this.deliver(event, context)).catch(this.options.report);
    });
  }

  async stop(): Promise<void> {
    this.unsubscribe?.();
    this.unsubscribe = null;
    await this.pending;
  }

  private async deliver(event: HarnessEvent, context: Context): Promise<void> {
    if (event.conversation === null) {
      this.options.report(new Error(`event ${event.id} from ${event.module} has no owning conversation; dropped`));
      return;
    }
    const result = await this.options.inputs.submit(
      event.conversation,
      await this.options.drafts.forEvent(event),
      context,
    );
    if (result.kind === "unknown_conversation") {
      this.options.report(
        new Error(`event ${event.id} names conversation ${event.conversation}, which does not exist; dropped`),
      );
    }
  }
}
