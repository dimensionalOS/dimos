import type { Refusal } from "#dimos/agents/shared/protocol/outcome.ts";

/** The one sentence each refusal becomes, in every frontend. Times are clock times in the viewer's zone. */
export function refusalText(refusal: Refusal): string {
  switch (refusal.code) {
    case "viewing":
      return "This view is read-only.";
    case "dimos_unavailable":
      return `DimOS is not reachable: ${refusal.detail}.`;
    case "module_failed":
      return `${refusal.op} ${refusal.module} failed: ${refusal.error}`;
    case "not_a_user_message":
      return "Only user messages can be edited; summarize works on any message.";
    case "nothing_queued":
      return "No user messages are queued.";
    case "running":
      return "A run is in progress; press Esc to stop it before changing the model.";
    case "not_found":
      return `No ${refusal.what} ${refusal.name}.`;
    case "invalid":
      return `${refusal.field}: ${refusal.problem}`;
    case "stale":
      return "The conversation moved on; try again.";
    case "not_running":
      return `Call ${refusal.call} is not running.`;
    case "login_failed":
      return `Login to ${refusal.provider} failed: ${refusal.error}`;
    case "model_failed":
      return `The model call failed: ${refusal.error}`;
    case "unknown_outcome":
      return "The gateway connection dropped during this; it may or may not have happened.";
    case "other_build":
      return refusal.gatewayBuild === null
        ? "The gateway runs another build; restart it with `dimos agent stop`, or with `systemctl --user restart dimos-agent.service` when it runs as the service."
        : `The gateway runs another build (${refusal.gatewayBuild}); restart it with \`dimos agent stop\`, or with \`systemctl --user restart dimos-agent.service\` when it runs as the service.`;
    case "stopping":
      return "The gateway is stopping.";
  }
}
