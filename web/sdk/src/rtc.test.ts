import { afterEach, describe, expect, it, vi } from "vitest";
import { GATHER_SETTLE_MS, GATHER_TIMEOUT_MS, type PeerConnectionLike, ViewerRtc } from "./rtc.ts";

/** A scriptable RTCPeerConnection: records calls, assigns mids to its
 * transceivers on the first remote offer, and lets tests fire ICE events. */
export class FakePeerConnection implements PeerConnectionLike {
  iceGatheringState = "new";
  connectionState = "new";
  localDescription: { sdp: string } | null = null;
  readonly remote: RTCSessionDescriptionInit[] = [];
  readonly transceivers: { mid: string | null; receiver: { track: MediaStreamTrack } }[] = [];
  readonly added: { kind: string; init?: { direction?: string } }[] = [];
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
        track: { id: `track-${this.transceivers.length}` } as unknown as MediaStreamTrack,
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

  candidate(type: string | null): void {
    const event = { candidate: type === null ? null : { type } } as unknown as Event;
    for (const listener of this.#listeners.get("icecandidate") ?? []) listener(event);
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

describe("ViewerRtc", () => {
  afterEach(() => {
    vi.useRealTimers();
  });

  it("offers one recvonly video transceiver and returns the gathered local SDP", async () => {
    const pc = new FakePeerConnection();
    const sdp = await new ViewerRtc(pc, () => {}).offer();
    expect(sdp).toBe(pc.offerSdp);
    expect(pc.added).toEqual([{ kind: "video", init: { direction: "recvonly" } }]);
  });

  it("sends the offer after the first srflx/relay candidate plus the settle window", async () => {
    vi.useFakeTimers();
    const pc = new FakePeerConnection();
    pc.onGather = () => {
      pc.candidate("host");
      pc.candidate("srflx");
      pc.candidate("relay"); // a sibling inside the window does not restart it
    };
    let offered = false;
    const offer = new ViewerRtc(pc, () => {}).offer().then(() => {
      offered = true;
    });
    await vi.advanceTimersByTimeAsync(GATHER_SETTLE_MS - 1);
    expect(offered).toBe(false);
    await vi.advanceTimersByTimeAsync(1);
    await offer;
    expect(offered).toBe(true);
  });

  it("host-only candidates wait for the cap (or end-of-candidates)", async () => {
    vi.useFakeTimers();
    const pc = new FakePeerConnection();
    pc.onGather = () => pc.candidate("host");
    let offered = false;
    const offer = new ViewerRtc(pc, () => {}).offer().then(() => {
      offered = true;
    });
    await vi.advanceTimersByTimeAsync(GATHER_TIMEOUT_MS - 1);
    expect(offered).toBe(false);
    await vi.advanceTimersByTimeAsync(1);
    await offer;
    expect(offered).toBe(true);

    const other = new FakePeerConnection();
    other.onGather = () => {
      other.candidate("host");
      other.candidate(null); // end-of-candidates
    };
    await expect(new ViewerRtc(other, () => {}).offer()).resolves.toBe(other.offerSdp);
  });

  it("accept applies the answer; renegotiate answers an SFU offer and exposes its tracks", async () => {
    const pc = new FakePeerConnection();
    const rtc = new ViewerRtc(pc, () => {});
    await rtc.offer();
    await rtc.accept("v=0\r\nsfu-answer\r\n");
    expect(pc.remote).toEqual([{ type: "answer", sdp: "v=0\r\nsfu-answer\r\n" }]);

    const answer = await rtc.renegotiate("v=0\r\nm=video\r\nm=video\r\n");
    expect(answer).toBe(pc.answerSdp);
    expect(pc.remote[1]).toEqual({ type: "offer", sdp: "v=0\r\nm=video\r\nm=video\r\n" });
    expect(rtc.trackFor("0")).toBe(pc.transceivers[0].receiver.track);
    expect(rtc.trackFor("1")).toBe(pc.transceivers[1].receiver.track);
    expect(rtc.trackFor("7")).toBeNull();
    rtc.close();
    expect(pc.closed).toBe(1);
  });
  it("routes a failed connection to the callback; disconnected and closed are not failures", () => {
    const pc = new FakePeerConnection();
    let failed = 0;
    new ViewerRtc(pc, () => failed++);
    pc.connection("disconnected");
    pc.connection("closed");
    expect(failed).toBe(0);
    pc.connection("failed");
    expect(failed).toBe(1);
  });
});
