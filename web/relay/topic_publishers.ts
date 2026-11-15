// Authentication/lifetime binding only. The provider owns expiry, safe-stop
// bytes and command tickets; this relay never interprets a robot command.
import {
  encodeDataFrame,
  type FrameHeader,
  MAX_TOPIC_GUARD_BYTES,
  MAX_TOPIC_GUARDS,
  type TopicCapabilities,
  topicGuardMeta,
  type TopicMsg,
} from "@dimos/shared";
import type { TopicProvider, TopicViewer } from "./topics.ts";

interface Source {
  id: string;
  gen: number;
  peer: TopicProvider;
  capabilities: TopicCapabilities;
  closing: boolean;
}
interface Publisher {
  viewer: TopicViewer;
  source: Source;
  id: string;
  handle: string;
  ready: boolean;
  stopping: boolean;
  deadline: number;
  sequence: number;
  pending: Set<number>;
}

export class TopicPublishers {
  #publishers = new Map<string, Publisher>();
  #owners = new WeakMap<TopicViewer, string>();
  constructor(readonly id: (prefix: string) => string, readonly now: () => number) {}

  publish(
    viewer: TopicViewer,
    source: Source | undefined,
    header: FrameHeader,
    payload: Uint8Array,
  ) {
    const meta = topicGuardMeta(header.meta);
    if (!meta || header.delivery !== "reliable" || payload.length > MAX_TOPIC_GUARD_BYTES) {
      viewer.sink.kick("invalid_guard_publication");
      return;
    }
    const reject = (code: string) => {
      if (meta.action === "open") {
        viewer.sendMsg({ t: "topic_error", id: meta.id, code, message: code, outcome: "rejected" });
      } else {
        viewer.sendMsg({
          t: "topic_guard_result",
          handle: meta.handle,
          gen: meta.gen,
          seq: header.seq,
          outcome: "rejected",
          code,
          providerTs: 0,
        });
      }
    };
    // `source` is undefined while the relay keeps topics closed (topics.ts).
    if (!source?.capabilities.write) {
      reject("topic_write_denied");
      return;
    }
    if (source.closing || !source.capabilities.guardedPublishers) {
      reject("guarded_publishers_unavailable");
      return;
    }
    if (source.gen !== meta.gen) {
      reject("stale_generation");
      return;
    }
    if (meta.action === "open") {
      const all = [...this.#publishers.values()];
      if (
        all.length >= 256 || all.filter((p) => p.source === source).length >= MAX_TOPIC_GUARDS ||
        all.filter((p) => p.viewer === viewer).length >= MAX_TOPIC_GUARDS ||
        all.some((p) => p.viewer === viewer && p.id === meta.id) || !payload.length
      ) {
        reject("guard_limit");
        return;
      }
      let owner = this.#owners.get(viewer);
      if (!owner) this.#owners.set(viewer, owner = this.id("owner"));
      const handle = this.id("@p:");
      this.#publishers.set(handle, {
        viewer,
        source,
        id: meta.id,
        handle,
        ready: false,
        stopping: false,
        deadline: this.now() + 10_000,
        sequence: 0,
        pending: new Set(),
      });
      source.peer.sendTopicGuard(encodeDataFrame({
        ...header,
        meta: { ...meta, id: handle, handle, owner },
      }, payload));
      return;
    }
    const publisher = this.#publishers.get(meta.handle);
    if (
      publisher?.viewer !== viewer || publisher.source !== source ||
      !publisher.ready || publisher.stopping ||
      !Number.isSafeInteger(header.seq) || header.seq <= publisher.sequence
    ) {
      reject("stale_guard_command");
      return;
    }
    // Successful write() is not delivery. Keep each command charged until
    // the provider accepts, supersedes or rejects it; never queue unboundedly.
    if (publisher.pending.size >= 8) {
      reject("guard_busy");
      return;
    }
    publisher.sequence = header.seq;
    publisher.pending.add(header.seq);
    source.peer.sendTopicGuard(encodeDataFrame(header, payload));
  }

  viewerMessage(viewer: TopicViewer, msg: TopicMsg): boolean {
    if (msg.t !== "topic_guard_stop" && msg.t !== "topic_guard_heartbeat") return false;
    const publisher = this.#publishers.get(msg.handle);
    if (publisher?.viewer !== viewer || publisher.source.gen !== msg.gen) return true;
    if (msg.t === "topic_guard_stop") this.#stop(publisher);
    else if (publisher.ready && !publisher.stopping) publisher.source.peer.sendControl(msg);
    return true;
  }

  providerMessage(peer: TopicProvider, msg: TopicMsg): void {
    if (
      msg.t !== "topic_guard_ready" && msg.t !== "topic_guard_ticket" &&
      msg.t !== "topic_guard_closed" && msg.t !== "topic_guard_result"
    ) return;
    const publisher = this.#publishers.get(msg.handle);
    if (publisher?.source.peer !== peer || publisher.source.gen !== msg.gen) return;
    if (msg.t === "topic_guard_ready") {
      if (publisher.ready || msg.id !== publisher.handle) return;
      publisher.ready = true;
      publisher.deadline = Infinity;
      if (publisher.stopping) this.#stop(publisher);
      else publisher.viewer.sendMsg({ ...msg, id: publisher.id });
    } else if (msg.t === "topic_guard_closed") {
      this.#publishers.delete(msg.handle);
      if (!publisher.ready) {
        publisher.viewer.sendMsg({
          t: "topic_error",
          id: publisher.id,
          code: msg.code,
          message: msg.code,
          outcome: "rejected",
        });
      } else publisher.viewer.sendMsg(msg);
    } else if (msg.t === "topic_guard_result") {
      if (publisher.pending.delete(msg.seq)) publisher.viewer.sendMsg(msg);
    } else if (!publisher.stopping) publisher.viewer.sendMsg(msg);
  }

  #stop(publisher: Publisher): void {
    publisher.stopping = true;
    // A control stop can overtake the binary open. Wait for its ack so it
    // cannot be ignored before creation and leave a late publisher alive.
    if (publisher.ready) {
      publisher.deadline = Math.min(publisher.deadline, this.now() + 2000);
      publisher.source.peer.sendControl({
        t: "topic_guard_stop",
        handle: publisher.handle,
        gen: publisher.source.gen,
      });
    }
  }

  viewerClosed(viewer: TopicViewer): void {
    for (const publisher of this.#publishers.values()) {
      if (publisher.viewer === viewer) this.#stop(publisher);
    }
  }

  sourceClosed(peer: TopicProvider): void {
    for (const [handle, publisher] of this.#publishers) {
      if (publisher.source.peer !== peer) continue;
      this.#publishers.delete(handle);
      if (publisher.ready) {
        publisher.viewer.sendMsg({
          t: "topic_guard_closed",
          handle,
          gen: publisher.source.gen,
          code: "source_offline",
        });
      } else {
        publisher.viewer.sendMsg({
          t: "topic_error",
          id: publisher.id,
          code: "source_offline",
          message: "source_offline",
          outcome: "unknown",
        });
      }
    }
  }

  reap(now: number): void {
    for (const publisher of this.#publishers.values()) {
      if (now >= publisher.deadline && !publisher.source.closing) {
        publisher.source.closing = true;
        publisher.source.peer.failTopics("guard_provider_timeout");
      }
    }
  }
}
