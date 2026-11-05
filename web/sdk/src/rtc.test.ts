import { afterEach, describe, expect, it, vi } from "vitest";
import { GATHER_SETTLE_MS, GATHER_TIMEOUT_MS, ViewerRtc } from "./rtc.ts";
import { FakePeerConnection } from "./testing/fakePeerConnection.ts";

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
