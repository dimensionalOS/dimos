import type { JsonValue, PanelSpec } from "@dimos/shared";
import type { Session } from "@dimos/sdk";

export function mapChannel(spec: PanelSpec, key: string): string | undefined {
  const value = spec.params[key];
  return typeof value === "string" ? value : undefined;
}

export function sendMapCommand(
  session: Session,
  ch: string,
  value: JsonValue,
  onError: (message: string | null) => void,
): void {
  session.publish(ch, value).then(
    () => onError(null),
    (err: unknown) => onError(`send failed: ${err instanceof Error ? err.message : String(err)}`),
  );
}
