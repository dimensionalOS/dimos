/** Checks JSON from HarnessRuntime against one shape, or throws WireError. Every wire shape implements it. */
export interface WireDecoder<T> {
  /** Names the shape in errors, e.g. "inventory result". */
  readonly name: string;
  decode(json: unknown): T;
}

export class WireError extends Error {
  readonly shape: string;
  readonly problems: readonly string[];

  constructor(shape: string, problems: readonly string[]) {
    super(`Invalid ${shape} from HarnessRuntime: ${problems.join("; ")}`);
    this.shape = shape;
    this.problems = problems;
  }
}
