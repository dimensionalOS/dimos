import type { Validator } from "typebox/compile";

/** The validator's first complaint about `json`, as `/points/0/1 must be number`. */
export function firstProblem(validator: Validator, json: unknown): string {
  const [error] = validator.Errors(json);
  return error === undefined ? "invalid" : `${error.instancePath || "/"} ${error.message}`;
}
