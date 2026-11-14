import type { PeerConnectionLike } from "../rtc.ts";

/** A remote track as a browser hands it over: muted until the first packet. */
class FakeTrack extends EventTarget {
  muted = true;
  constructor(readonly id: string) {
    super();
  }
  unmute(): void {
    this.muted = false;
    this.dispatchEvent(new Event("unmute"));
  }
}

/** A scriptable RTCPeerConnection: records calls, assigns mids to its
 * transceivers on the first remote offer, and lets tests fire ICE events. */
export class FakePeerConnection implements PeerConnectionLike {
  iceGatheringState = "new";
  connectionState = "new";
  localDescription: { sdp: string } | null = null;
  readonly remote: RTCSessionDescriptionInit[] = [];
  readonly transceivers: { mid: string | null; receiver: { track: MediaStreamTrack } }[] = [];
  readonly added: { kind: string; init?: { direction?: string } }[] = [];
  readonly remoteCandidates: ({ candidate: string; sdpMid: string | null } | null)[] = [];
  closed = 0;
  offerSdp = "v=0\r\noffer\r\n";
  answerSdp = "v=0\r\nanswer\r\n";
  /** Fired by setLocalDescription(offer): the gathering events to emit. */
  onGather: (() => void) | null = () => this.complete();
  #listeners = new Map<string, ((event: Event) => void)[]>();
  #nextMid = 0;

  addTransceiver(kind: string, init?: { direction?: string }): unknown {
    this.added.push({ kind, init });
    const transceiver = {
      mid: null,
      receiver: {
        track: new FakeTrack(`track-${this.transceivers.length}`) as unknown as MediaStreamTrack,
      },
    };
    this.transceivers.push(transceiver);
    return transceiver;
  }

  createOffer(): Promise<RTCSessionDescriptionInit> {
    return Promise.resolve({ type: "offer", sdp: this.offerSdp });
  }

  createAnswer(): Promise<RTCSessionDescriptionInit> {
    return Promise.resolve({ type: "answer", sdp: this.answerSdp });
  }

  setLocalDescription(description?: RTCSessionDescriptionInit): Promise<void> {
    this.localDescription = { sdp: description?.sdp ?? "" };
    if (description?.type === "offer") this.onGather?.();
    return Promise.resolve();
  }

  setRemoteDescription(description: RTCSessionDescriptionInit): Promise<void> {
    this.remote.push(description);
    if (description.type === "offer") {
      // An SFU offer names every m-section: a pull lands on the first free
      // transceiver (or adds one), mids are assigned in order.
      const pulls = (description.sdp?.match(/m=video/g) ?? []).length;
      while (this.transceivers.length < pulls) this.addTransceiver("video");
      for (const t of this.transceivers) if (t.mid === null) t.mid = `${this.#nextMid++}`;
    }
    return Promise.resolve();
  }

  addIceCandidate(candidate: { candidate: string; sdpMid: string | null } | null): Promise<void> {
    this.remoteCandidates.push(candidate);
    return Promise.resolve();
  }

  getTransceivers(): { mid: string | null; receiver: { track: MediaStreamTrack } }[] {
    return this.transceivers;
  }

  addEventListener(type: string, listener: (event: Event) => void): void {
    const list = this.#listeners.get(type) ?? [];
    list.push(listener);
    this.#listeners.set(type, list);
  }

  close(): void {
    this.closed++;
  }

  candidate(type: string | null, line = `candidate:1 1 udp 1 10.0.0.2 5000 typ ${type}`): void {
    const candidate = type === null ? null : { type, candidate: line, sdpMid: "0" };
    const event = { candidate } as unknown as Event;
    for (const listener of this.#listeners.get("icecandidate") ?? []) listener(event);
  }

  /** The provider's track arrived on the first transceiver. */
  track(): MediaStreamTrack {
    if (this.transceivers.length === 0) this.addTransceiver("video");
    const transceiver = this.transceivers[0];
    transceiver.mid ??= "0";
    const event = {
      track: transceiver.receiver.track,
      receiver: transceiver.receiver,
    } as unknown as Event;
    for (const listener of this.#listeners.get("track") ?? []) listener(event);
    return transceiver.receiver.track;
  }

  /** The first packet of the provider's track arrived. */
  media(): void {
    (this.transceivers[0].receiver.track as unknown as FakeTrack).unmute();
  }

  complete(): void {
    this.iceGatheringState = "complete";
    for (const listener of this.#listeners.get("icegatheringstatechange") ?? []) {
      listener({} as Event);
    }
  }

  connection(state: string): void {
    this.connectionState = state;
    for (const listener of this.#listeners.get("connectionstatechange") ?? []) {
      listener({} as Event);
    }
  }
}
