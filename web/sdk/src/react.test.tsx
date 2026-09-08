// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { useChannel, useStatus, useStoreChannel } from "./react.ts";
import { connect, type Session } from "./session.ts";
import { type ChannelStore, StatusStore } from "./store.ts";
import { FakeRelayEnd, INFO, manifest, ROBOT_A, settle, spec, until } from "./testing/fakeRelay.ts";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

function OdomSeq({ session }: { session: Session }) {
  const snapshot = useChannel(session, "odom");
  return <span>{snapshot.slot?.seq ?? "none"}</span>;
}

function Phase({ session }: { session: { status: StatusStore } }) {
  const status = useStatus(session);
  return <span>{status.transport.phase}</span>;
}

function StoreSeq({ store }: { store: ChannelStore }) {
  const snapshot = useStoreChannel(store, "odom");
  return <span>{snapshot.slot?.seq ?? "none"}</span>;
}

describe("react hooks", () => {
  let container: HTMLElement;
  let root: Root;
  const handles: Session[] = [];

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
    for (const handle of handles.splice(0)) handle.close();
  });

  /** A live session against a fake relay, with the odom manifest adopted and
   * the UI ticker parked (tests publish snapshots by hand inside act()). */
  async function live(): Promise<{ relay: FakeRelayEnd; session: Session }> {
    const relay = new FakeRelayEnd();
    const session = connect({ uiTickMs: 3_600_000 }, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    handles.push(session);
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");
    relay.pushManifest("a", manifest([spec()]));
    await until(() => session.status.get().manifest !== null, "manifest");
    return { relay, session };
  }

  it("useChannel acquires wire interest while mounted and releases on unmount", async () => {
    const { relay, session } = await live();
    act(() => root.render(<OdomSeq session={session} />));
    await until(() => relay.subs().length === 1, "sub");
    expect(relay.subs()).toEqual(["odom"]);
    expect(container.textContent).toBe("none");

    relay.pushFrame(3, { x: 1.5 });
    await until(() => session.store.get("odom")?.seq === 3, "frame");
    act(() => session.store.publishUi());
    expect(container.textContent).toBe("3");

    act(() => root.render(null));
    await until(() => relay.unsubs().length === 1, "unsub");
    expect(relay.unsubs()).toEqual(["odom"]);
  });

  it("useStatus re-renders on status updates", () => {
    const status = new StatusStore();
    act(() => root.render(<Phase session={{ status }} />));
    expect(container.textContent).toBe("connecting");

    act(() => status.update({ transport: { phase: "connected" } }));
    expect(container.textContent).toBe("connected");
  });

  it("useStoreChannel observes the store without creating wire interest", async () => {
    const { relay, session } = await live();
    act(() => root.render(<StoreSeq store={session.store} />));
    relay.pushFrame(5, { x: 1.5 });
    await until(() => session.store.get("odom")?.seq === 5, "frame");
    act(() => session.store.publishUi());
    expect(container.textContent).toBe("5");

    act(() => root.render(null));
    await settle();
    expect(relay.subs()).toEqual([]);
    expect(relay.unsubs()).toEqual([]);
  });
});
