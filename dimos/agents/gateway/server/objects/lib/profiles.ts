import type { Cell, Self } from "@earendil-works/chord";
import type { Harness } from "@earendil-works/pi-durable";
import type { Principal } from "#dimos/agents/gateway/auth/principal.ts";
import type { ProfileCatalog } from "#dimos/agents/shared/protocol/profiles.ts";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { GatewayState } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { ProfilesDoc, catalogFromSaved, profileCatalog } from "#dimos/agents/harness/profiles.ts";
import { commitEvents, valueOf } from "#dimos/agents/gateway/server/objects/lib/commits.ts";

export function profileRows(self: Self<Principal>, harness: Harness): Cell<ProfileCatalog> {
  const events = self.resource("profileEvents", {
    version: 1,
    open: () => commitEvents(harness),
    close: (events) => events.close(),
  });
  const rows = self.state("profiles", GatewayState.profiles, {
    init: (ctx) => profileCatalog(harness, withAbortSignal(ctx.signal, BACKGROUND_CONTEXT)),
  });
  self.on(
    "profileCommits",
    events,
    (events) => events,
    (publication, ctx) => {
      for (const change of publication.changes) {
        if (change.type === "document" && change.record.kind === ProfilesDoc.definition.kind)
          rows.replace(ctx, catalogFromSaved(change.value === null ? null : valueOf(change.value, ProfilesDoc)));
      }
    },
  );
  return rows;
}
