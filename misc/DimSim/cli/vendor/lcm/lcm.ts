// LCM Main Class - Pure TypeScript Implementation (vendored from @dimos/lcm@0.2.0)

import type {
  LCMOptions,
  LCMMessage,
  MessageClass,
  ParsedUrl,
  Subscription,
  SubscriptionHandler,
  PacketHandler,
  PacketSubscription,
} from "./types.ts";
import { MAX_SMALL_MESSAGE, SHORT_HEADER_SIZE } from "./types.ts";
import { parseUrl } from "./url.ts";
import {
  UdpMulticastSocket,
  FragmentReassembler,
  encodeSmallMessage,
  encodeFragmentedMessage,
  decodePacket,
} from "./transport.ts";

const textEncoder = new TextEncoder();

export class LCM {
  private readonly config: ParsedUrl;
  private socket: UdpMulticastSocket | null = null;
  private reassembler = new FragmentReassembler();
  private subscriptions: Subscription[] = [];
  private packetSubscriptions: PacketSubscription[] = [];
  private sequenceNumber = 0;
  private running = false;
  private messageQueue: LCMMessage<Uint8Array>[] = [];

  constructor(url?: string);
  constructor(options?: LCMOptions);
  constructor(urlOrOptions?: string | LCMOptions) {
    if (typeof urlOrOptions === "string") {
      this.config = parseUrl(urlOrOptions);
    } else {
      this.config = parseUrl(urlOrOptions?.url);
      if (urlOrOptions?.ttl !== undefined) {
        this.config.ttl = urlOrOptions.ttl;
      }
      if (urlOrOptions?.iface !== undefined) {
        this.config.iface = urlOrOptions.iface;
      }
    }
  }

  async start(): Promise<void> {
    if (this.running) return;
    this.socket = new UdpMulticastSocket(this.config);
    this.running = true;
    await this.socket.listen((data, _addr) => {
      this.handlePacket(data);
    });
  }

  stop(): void {
    this.running = false;
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }

  subscribeRaw(channelPattern: string, handler: SubscriptionHandler<Uint8Array>): () => void {
    const pattern = this.channelToRegex(channelPattern);
    const subscription: Subscription = {
      channel: channelPattern,
      pattern,
      handler: handler as SubscriptionHandler<unknown>,
    };
    this.subscriptions.push(subscription);
    return () => {
      const idx = this.subscriptions.indexOf(subscription);
      if (idx !== -1) this.subscriptions.splice(idx, 1);
    };
  }

  subscribePacket(handler: PacketHandler): () => void;
  subscribePacket(channelPattern: string, handler: PacketHandler): () => void;
  subscribePacket(patternOrHandler: string | PacketHandler, maybeHandler?: PacketHandler): () => void {
    const pattern = typeof patternOrHandler === "string"
      ? this.channelToRegex(patternOrHandler)
      : null;
    const handler = typeof patternOrHandler === "function"
      ? patternOrHandler
      : maybeHandler!;
    const subscription: PacketSubscription = { pattern, handler };
    this.packetSubscriptions.push(subscription);
    return () => {
      const idx = this.packetSubscriptions.indexOf(subscription);
      if (idx !== -1) this.packetSubscriptions.splice(idx, 1);
    };
  }

  subscribe<T>(channel: string, msgClass: MessageClass<T>, handler: SubscriptionHandler<T>): () => void {
    const typeName = (msgClass as unknown as { _NAME: string })._NAME;
    const fullChannel = channel.includes("#") ? channel : `${channel}#${typeName}`;
    const pattern = this.channelToRegex(fullChannel);
    const subscription: Subscription = {
      channel: fullChannel,
      pattern,
      handler: handler as SubscriptionHandler<unknown>,
      msgClass: msgClass as MessageClass<unknown>,
    };
    this.subscriptions.push(subscription);
    return () => {
      const idx = this.subscriptions.indexOf(subscription);
      if (idx !== -1) this.subscriptions.splice(idx, 1);
    };
  }

  async publishRaw(channel: string, data: Uint8Array): Promise<void> {
    if (!this.socket) throw new Error("LCM not started. Call start() first.");
    const channelBytes = textEncoder.encode(channel);
    const totalSize = SHORT_HEADER_SIZE + channelBytes.length + 1 + data.length;
    const seq = this.sequenceNumber++;
    if (totalSize <= MAX_SMALL_MESSAGE) {
      const packet = encodeSmallMessage(channel, data, seq);
      await this.socket.send(packet);
    } else {
      const fragments = encodeFragmentedMessage(channel, data, seq);
      for (const fragment of fragments) {
        await this.socket.send(fragment);
      }
    }
  }

  async publish<T extends { encode(): Uint8Array }>(channel: string, msg: T): Promise<void> {
    const data = msg.encode();
    const typeName = (msg.constructor as unknown as { _NAME?: string })._NAME;
    const fullChannel = typeName && !channel.includes("#")
      ? `${channel}#${typeName}`
      : channel;
    await this.publishRaw(fullChannel, data);
  }

  async publishPacket(packet: Uint8Array): Promise<void> {
    if (!this.socket) throw new Error("LCM not started. Call start() first.");
    await this.socket.send(packet);
  }

  handle(timeoutMs: number = 0): number {
    const messages = this.messageQueue.splice(0);
    for (const msg of messages) {
      this.dispatchMessage(msg);
    }
    return messages.length;
  }

  async handleAsync(timeoutMs: number = 100): Promise<number> {
    const startTime = Date.now();
    while (this.messageQueue.length === 0) {
      if (timeoutMs >= 0 && Date.now() - startTime >= timeoutMs) break;
      await new Promise((resolve) => setTimeout(resolve, 10));
    }
    return this.handle();
  }

  async run(callback?: () => void | Promise<void>): Promise<void> {
    while (this.running) {
      await this.handleAsync(100);
      if (callback) await callback();
    }
  }

  private channelToRegex(pattern: string): RegExp {
    const escaped = pattern.replace(/[.+?^${}()|[\]\\]/g, "\\$&");
    const regexStr = "^" + escaped.replace(/\*/g, ".*") + "$";
    return new RegExp(regexStr);
  }

  private handlePacket(data: Uint8Array): void {
    const decoded = decodePacket(data);
    if (!decoded) return;

    const channel = decoded.type === "small" ? decoded.channel : decoded.channel;

    if (channel) {
      for (const sub of this.packetSubscriptions) {
        if (!sub.pattern || sub.pattern.test(channel)) {
          try { sub.handler(data); } catch (e) { console.error(`Error in raw packet handler:`, e); }
        }
      }
    }

    if (decoded.type === "small") {
      this.queueMessage(decoded.channel, decoded.data);
    } else {
      const complete = this.reassembler.processFragment(decoded);
      if (complete) this.queueMessage(complete.channel, complete.data);
    }
  }

  private queueMessage(channel: string, data: Uint8Array): void {
    const msg: LCMMessage<Uint8Array> = {
      channel,
      data: new Uint8Array(data),
      timestamp: Date.now(),
    };
    this.messageQueue.push(msg);
  }

  private dispatchMessage(msg: LCMMessage<Uint8Array>): void {
    for (const sub of this.subscriptions) {
      if (sub.pattern.test(msg.channel)) {
        try {
          if (sub.msgClass) {
            const decoded = sub.msgClass.decode(msg.data);
            sub.handler({ channel: msg.channel, data: decoded, timestamp: msg.timestamp });
          } else {
            sub.handler(msg);
          }
        } catch (e) {
          console.error(`Error in subscription handler for ${msg.channel}:`, e);
        }
      }
    }
  }

  getConfig(): ParsedUrl {
    return { ...this.config };
  }

  isRunning(): boolean {
    return this.running;
  }

  /** Peek at the next sequence number (for echo filtering). */
  getNextSeq(): number {
    return this.sequenceNumber;
  }
}
