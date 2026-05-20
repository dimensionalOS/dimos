// LCM UDP Multicast Transport (vendored from @dimos/lcm@0.2.0)
// FIX: Added joinMulticastV4() call in UdpMulticastSocket.listen()

import {
  MAGIC_SHORT,
  MAGIC_LONG,
  MAX_SMALL_MESSAGE,
  SHORT_HEADER_SIZE,
  FRAGMENT_HEADER_SIZE,
} from "./types.ts";
import type { ParsedUrl } from "./types.ts";

const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

/** Encode a small LCM message (fits in single UDP packet) */
export function encodeSmallMessage(
  channel: string,
  data: Uint8Array,
  sequenceNumber: number
): Uint8Array {
  const channelBytes = textEncoder.encode(channel);
  const totalSize = SHORT_HEADER_SIZE + channelBytes.length + 1 + data.length;

  if (totalSize > MAX_SMALL_MESSAGE) {
    throw new Error(`Message too large for small message format: ${totalSize} > ${MAX_SMALL_MESSAGE}`);
  }

  const buffer = new Uint8Array(totalSize);
  const view = new DataView(buffer.buffer);

  let offset = 0;

  // Magic number (big-endian)
  view.setUint32(offset, MAGIC_SHORT, false);
  offset += 4;

  // Sequence number (big-endian)
  view.setUint32(offset, sequenceNumber, false);
  offset += 4;

  // Channel name (null-terminated)
  buffer.set(channelBytes, offset);
  offset += channelBytes.length;
  buffer[offset] = 0; // null terminator
  offset += 1;

  // Payload
  buffer.set(data, offset);

  return buffer;
}

/** Encode a fragmented LCM message (requires multiple UDP packets) */
export function encodeFragmentedMessage(
  channel: string,
  data: Uint8Array,
  sequenceNumber: number,
  maxFragmentSize: number = 65000
): Uint8Array[] {
  const channelBytes = textEncoder.encode(channel);
  const payloadSize = data.length;

  const firstFragmentPayloadSpace = maxFragmentSize - FRAGMENT_HEADER_SIZE - channelBytes.length - 1;
  const subsequentFragmentPayloadSpace = maxFragmentSize - FRAGMENT_HEADER_SIZE;

  let numFragments = 1;
  let remainingBytes = payloadSize - Math.min(payloadSize, firstFragmentPayloadSpace);
  if (remainingBytes > 0) {
    numFragments += Math.ceil(remainingBytes / subsequentFragmentPayloadSpace);
  }

  const fragments: Uint8Array[] = [];
  let payloadOffset = 0;

  for (let fragmentNum = 0; fragmentNum < numFragments; fragmentNum++) {
    const isFirst = fragmentNum === 0;
    const headerSize = FRAGMENT_HEADER_SIZE;
    const channelSize = isFirst ? channelBytes.length + 1 : 0;

    const maxPayloadForThisFragment = isFirst
      ? firstFragmentPayloadSpace
      : subsequentFragmentPayloadSpace;

    const payloadForThisFragment = Math.min(
      maxPayloadForThisFragment,
      payloadSize - payloadOffset
    );

    const fragmentSize = headerSize + channelSize + payloadForThisFragment;
    const fragment = new Uint8Array(fragmentSize);
    const view = new DataView(fragment.buffer);

    let offset = 0;

    view.setUint32(offset, MAGIC_LONG, false);
    offset += 4;
    view.setUint32(offset, sequenceNumber, false);
    offset += 4;
    view.setUint32(offset, payloadSize, false);
    offset += 4;
    view.setUint32(offset, payloadOffset, false);
    offset += 4;
    view.setUint16(offset, fragmentNum, false);
    offset += 2;
    view.setUint16(offset, numFragments, false);
    offset += 2;

    if (isFirst) {
      fragment.set(channelBytes, offset);
      offset += channelBytes.length;
      fragment[offset] = 0;
      offset += 1;
    }

    fragment.set(data.subarray(payloadOffset, payloadOffset + payloadForThisFragment), offset);
    payloadOffset += payloadForThisFragment;
    fragments.push(fragment);
  }

  return fragments;
}

/** Decoded small message */
export interface DecodedSmallMessage {
  type: "small";
  channel: string;
  data: Uint8Array;
  sequenceNumber: number;
}

/** Decoded fragment */
export interface DecodedFragment {
  type: "fragment";
  sequenceNumber: number;
  payloadSize: number;
  fragmentOffset: number;
  fragmentNumber: number;
  numFragments: number;
  channel?: string;
  data: Uint8Array;
}

/** Decode a received UDP packet */
export function decodePacket(packet: Uint8Array): DecodedSmallMessage | DecodedFragment | null {
  if (packet.length < SHORT_HEADER_SIZE) {
    return null;
  }

  const view = new DataView(packet.buffer, packet.byteOffset, packet.byteLength);
  const magic = view.getUint32(0, false);

  if (magic === MAGIC_SHORT) {
    return decodeSmallPacket(packet, view);
  } else if (magic === MAGIC_LONG) {
    return decodeFragmentPacket(packet, view);
  }

  return null;
}

function decodeSmallPacket(packet: Uint8Array, view: DataView): DecodedSmallMessage | null {
  const sequenceNumber = view.getUint32(4, false);

  let channelEnd = SHORT_HEADER_SIZE;
  while (channelEnd < packet.length && packet[channelEnd] !== 0) {
    channelEnd++;
  }

  if (channelEnd >= packet.length) {
    return null;
  }

  const channel = textDecoder.decode(packet.subarray(SHORT_HEADER_SIZE, channelEnd));
  const data = packet.subarray(channelEnd + 1);

  return { type: "small", channel, data, sequenceNumber };
}

function decodeFragmentPacket(packet: Uint8Array, view: DataView): DecodedFragment | null {
  if (packet.length < FRAGMENT_HEADER_SIZE) {
    return null;
  }

  const sequenceNumber = view.getUint32(4, false);
  const payloadSize = view.getUint32(8, false);
  const fragmentOffset = view.getUint32(12, false);
  const fragmentNumber = view.getUint16(16, false);
  const numFragments = view.getUint16(18, false);

  let offset = FRAGMENT_HEADER_SIZE;
  let channel: string | undefined;

  if (fragmentNumber === 0) {
    let channelEnd = offset;
    while (channelEnd < packet.length && packet[channelEnd] !== 0) {
      channelEnd++;
    }
    if (channelEnd >= packet.length) {
      return null;
    }
    channel = textDecoder.decode(packet.subarray(offset, channelEnd));
    offset = channelEnd + 1;
  }

  const data = packet.subarray(offset);

  return { type: "fragment", sequenceNumber, payloadSize, fragmentOffset, fragmentNumber, numFragments, channel, data };
}

/** Fragment reassembler for handling large messages */
export class FragmentReassembler {
  private pending = new Map<number, {
    channel: string;
    payloadSize: number;
    numFragments: number;
    receivedFragments: Set<number>;
    buffer: Uint8Array;
    lastActivity: number;
  }>();

  private timeoutMs: number;

  constructor(timeoutMs: number = 5000) {
    this.timeoutMs = timeoutMs;
  }

  processFragment(fragment: DecodedFragment): { channel: string; data: Uint8Array } | null {
    const now = Date.now();
    this.cleanup(now);

    let entry = this.pending.get(fragment.sequenceNumber);

    if (!entry) {
      if (fragment.fragmentNumber !== 0 || !fragment.channel) {
        return null;
      }

      entry = {
        channel: fragment.channel,
        payloadSize: fragment.payloadSize,
        numFragments: fragment.numFragments,
        receivedFragments: new Set(),
        buffer: new Uint8Array(fragment.payloadSize),
        lastActivity: now,
      };
      this.pending.set(fragment.sequenceNumber, entry);
    }

    if (fragment.fragmentOffset + fragment.data.length > entry.buffer.length) {
      // Fragment doesn't fit — corrupted or mismatched packet, discard.
      this.pending.delete(fragment.sequenceNumber);
      return null;
    }
    entry.buffer.set(fragment.data, fragment.fragmentOffset);
    entry.receivedFragments.add(fragment.fragmentNumber);
    entry.lastActivity = now;

    if (entry.receivedFragments.size === entry.numFragments) {
      this.pending.delete(fragment.sequenceNumber);
      return { channel: entry.channel, data: entry.buffer };
    }

    return null;
  }

  private cleanup(now: number): void {
    for (const [seq, entry] of this.pending) {
      if (now - entry.lastActivity > this.timeoutMs) {
        this.pending.delete(seq);
      }
    }
  }
}

/** UDP Multicast socket wrapper for Deno */
export class UdpMulticastSocket {
  private socket: Deno.DatagramConn | null = null;
  private readonly config: ParsedUrl;
  private running = false;

  constructor(config: ParsedUrl) {
    this.config = config;
  }

  /** Start listening for multicast messages */
  async listen(onMessage: (data: Uint8Array, addr: Deno.NetAddr) => void): Promise<void> {
    // reuseAddress allows multiple processes to bind to the same multicast port
    this.socket = Deno.listenDatagram({
      port: this.config.port,
      transport: "udp",
      hostname: "0.0.0.0",
      reuseAddress: true,
    });

    // FIX: Join the multicast group and enable loopback for local IPC
    const membership = await this.socket.joinMulticastV4(this.config.host, this.config.iface ?? "0.0.0.0");
    membership.setLoopback(true);
    if (this.config.ttl !== undefined) {
      membership.setTTL(this.config.ttl);
    }

    this.running = true;

    // Read loop
    (async () => {
      try {
        while (this.running && this.socket) {
          const [data, addr] = await this.socket.receive();
          if (addr.transport === "udp") {
            onMessage(data, addr);
          }
        }
      } catch (e) {
        if (this.running) {
          console.error("UDP receive error:", e);
        }
      }
    })();
  }

  /** Send a UDP packet to the multicast group */
  async send(data: Uint8Array): Promise<void> {
    if (!this.socket) {
      // Create a socket for sending if we don't have one
      this.socket = Deno.listenDatagram({
        port: 0, // Ephemeral port for sending
        transport: "udp",
        hostname: "0.0.0.0",
      });
    }

    await this.socket.send(data, {
      transport: "udp",
      hostname: this.config.host,
      port: this.config.port,
    });
  }

  /** Close the socket */
  close(): void {
    this.running = false;
    if (this.socket) {
      try {
        this.socket.close();
      } catch {
        // Ignore close errors
      }
      this.socket = null;
    }
  }
}
