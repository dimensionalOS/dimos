import type { ChannelSpec, Session } from "@dimos/sdk";
import type { Inventory } from "../shared/protocol/runtime/inventory.ts";
import type { StreamRef } from "../shared/protocol/streams.ts";
import { StreamRefSchema } from "../shared/protocol/streams.ts";
import { connect, TRACK_ENCODING } from "@dimos/sdk";
import { WebTransport, quicheLoaded } from "@fails-components/webtransport";
import { Value } from "typebox/value";

export type RelayEndpoint = NonNullable<Inventory["environment"]["relay"]>;

export async function openRelay(endpoint: RelayEndpoint): Promise<Session> {
  await quicheLoaded;
  return connect(endpoint, {
    createWebTransport(info) {
      const transport = new WebTransport(`${info.wtUrl}/viewer`, {
        requireUnreliable: true,
        serverCertificateHashes: info.certHash
          ? [{ algorithm: "sha-256", value: Buffer.from(info.certHash, "base64") }]
          : [],
      });
      let closed = false;
      return {
        ready: transport.ready,
        closed: transport.closed,
        close() {
          if (closed) return;
          closed = true;
          transport.close();
          // webtransport 1.6.11 leaves its UDP client alive on a local close.
          const hook = transport.sessionint.closeHook;
          transport.sessionint.closeHook = null;
          hook?.();
        },
        createBidirectionalStream: () => transport.createBidirectionalStream(),
        incomingUnidirectionalStreams: transport.incomingUnidirectionalStreams,
        datagrams: { writable: transport.datagrams.createWritable() },
      };
    },
  });
}

export function channelFor(session: Session, channel: string): ChannelSpec | undefined {
  const spec = session.status.get().manifest?.channels.find((spec) => spec.ch === channel && spec.dir === "rx");
  if (spec?.encoding === TRACK_ENCODING)
    throw new Error(`Image previews in Node require rtc=false on the relay bridge (${spec.ch})`);
  return spec;
}

export function relayStreams(session: Session, inventory: Inventory): readonly StreamRef[] {
  return (session.status.get().manifest?.channels ?? [])
    .filter((spec) => spec.dir === "rx")
    .map((spec) => {
      const source = inventory.runtime.streams.find(
        (stream) =>
          (stream.direction === "out" || stream.direction === "inout") && `${stream.module}.${stream.name}` === spec.ch,
      );
      return Value.Parse(StreamRefSchema, {
        channel: spec.ch,
        encoding: spec.encoding,
        source:
          source === undefined
            ? null
            : {
                module: source.module,
                name: source.name,
                direction: source.direction,
                type: source.type,
              },
      });
    });
}
