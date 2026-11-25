import type { Readable } from "@earendil-works/chord";

/** What a reader holds of a changing value: Chord's read shape, so a Status and a Chord state member read alike. */
export type Live<T> = Pick<Readable<T>, "value" | "subscribe">;

/** A changing value, set by the code that owns it and read by everyone else as a Live. */
export class Status<T> implements Live<T> {
  private current: T;
  private readonly listeners = new Set<(value: T) => void>();
  /** Where a throwing listener's error goes; the other listeners still run. One that rethrows stops at the first. */
  private readonly report: (error: unknown) => void;

  constructor(initial: T, report: (error: unknown) => void) {
    this.current = initial;
    this.report = report;
  }

  get value(): T {
    return this.current;
  }

  /** Replaces the value and tells every listener; an identical value tells nobody. */
  set(value: T): void {
    if (Object.is(value, this.current)) return;
    this.current = value;
    for (const listener of [...this.listeners]) this.tell(listener, value);
  }

  /** Calls the listener with the current value now, then with every new value; returns the unsubscribe. */
  subscribe(listener: (value: T) => void): () => void {
    this.listeners.add(listener);
    this.tell(listener, this.current);
    return () => {
      this.listeners.delete(listener);
    };
  }

  private tell(listener: (value: T) => void, value: T): void {
    try {
      listener(value);
    } catch (error) {
      this.report(error);
    }
  }
}
