import { type CSSProperties, useEffect, useRef, useState } from "react";
import { connect, type Session } from "@dimos/sdk";
import { useStatus } from "@dimos/sdk/react";
import { Workspace } from "./cockpit/Workspace.tsx";
import { App } from "@dimos/cockpit/App.tsx";
import { cockpitDecoders, installAutoSubscriptions } from "@dimos/cockpit/subscriptions.ts";
import { transportForSession } from "./publicTransport.ts";
import { BallCameraPanel } from "./BallCameraPanel.tsx";
import { WorldPanel } from "./WorldPanel.tsx";
import { ViewerSession } from "./viewerSession.ts";
import { duckColors, RoomPreview } from "./RoomPreview.tsx";
import { useLobbyPreview } from "./lobbyPreview.ts";
import { HoverPreview } from "./hoverPreview.tsx";
import { playerName, robotIds, roster, teams } from "./roster.ts";
import styles from "./lobby.module.css";
import { PlayerDirectory, type PlayerIdentity } from "./players.ts";

interface Slot extends PlayerIdentity {
  id: string;
  occupied: boolean;
  connected: boolean;
  mine: boolean;
  reconnectSeconds: number;
}
interface Entry {
  token: string;
  role: "observe" | "visitor" | "host";
  robot: string;
  runtime: string;
  displayName: string | null;
  slots: Slot[];
  graceSeconds: number;
}
class LobbyError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
  }
}
async function request(
  action: string,
  token?: string,
  robot?: string,
  displayName?: string,
): Promise<Entry> {
  const response = await fetch("/api/lobby", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ action, token, robot, displayName }),
    signal: AbortSignal.timeout(8000),
  });
  const data = await response.json();
  if (!response.ok) {
    throw new LobbyError(data.error ?? "Could not connect to the world.", response.status);
  }
  return data;
}

function useWorldSession(entry: Entry | null): Session | null {
  const key = entry ? `${entry.token}:${entry.runtime}:${entry.role}` : "";
  const [current, setCurrent] = useState<
    { key: string; session: Session } | null
  >(null);
  useEffect(() => {
    if (!entry) return;
    const session = connect({
      url: `${location.origin}/sessions/${entry.token}`,
      robot: entry.runtime,
      decoders: cockpitDecoders,
    }, transportForSession(() => {
      // Replacement is intentional ownership transfer, not a network failure.
      session.close();
      globalThis.dispatchEvent(new Event("microduck-session-replaced"));
    }));
    const unsubscribe = entry.role === "observe"
      ? session.subscribe("world_state", () => {})
      : installAutoSubscriptions(session);
    setCurrent({ key, session });
    return () => {
      unsubscribe();
      session.close();
    };
    // A poll returns a new Entry but does not change the connection identity.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key]);
  return current?.key === key ? current.session : null;
}

function World({ entry, session, preview }: {
  entry: Entry;
  session: Session;
  preview: ReturnType<typeof useLobbyPreview>;
}) {
  const status = useStatus(session);
  if (!status.manifest) {
    return (
      <div className={styles.loading} role="status">
        <span className={styles.loadingMark}>◌</span>
        <h2>
          {entry.role === "observe"
            ? "Opening the world…"
            : `Getting ${entry.displayName || playerName(entry.robot)} ready…`}
        </h2>
        <p>
          {entry.role === "visitor"
            ? "Preparing your camera, controls and private agent."
            : "Connecting you to the football club."}
        </p>
      </div>
    );
  }
  return (
    <PlayerDirectory.Provider value={entry.slots}>
      <ViewerSession.Provider value={session}>
        <Workspace>
        {entry.role === "observe"
          ? (
            <div className={styles.observer}>
              <div className={`${styles.observerWorld} mw-world-stage`}>
                <WorldPanel
                  store={session.store}
                  spec={{
                    id: "observer-world",
                    kind: "world3d",
                    title: "Microduck World",
                    channels: ["world_state"],
                    params: {
                      view: "world",
                      robot: "duck1",
                      jpeg: "world_compare_image",
                    },
                  }}
                />
                <div className="mw-yolo-overlay">
                <BallCameraPanel
                  store={session.store}
                  spec={{
                    id: "spectator-camera",
                    kind: "ball-camera",
                    title: "YOLO",
                    channels: [],
                    params: { selectable: true, robot: "duck1" },
                  }}
                />
                </div>
              </div>
              <aside className={styles.worldGuide} aria-label="World overview">
                <span className={styles.eyebrow}>FOOTBALL CLUB</span>
                <RoomPreview
                  model={preview.model}
                  snapshot={preview.snapshot}
                />
                <h2>Six ducks. One pitch.</h2>
                <p>
                  Drag to orbit. Scroll to zoom. Follow their adventures from any angle.
                </p>
                <ul>
                  {Object.keys(duckColors).map((id) => (
                    <li key={id}>
                      <span
                        className={styles.dot}
                        style={{ background: duckColors[id] }}
                      />
                      {entry.slots.find((s) => s.id === id)?.displayName ||
                        playerName(id)}
                      <small>
                        {entry.slots.find((s) => s.id === id)?.occupied ? "Playing" : "Available"}
                      </small>
                    </li>
                  ))}
                </ul>
              </aside>
            </div>
          )
          : <App session={session} />}
      </Workspace>
      </ViewerSession.Provider>
    </PlayerDirectory.Provider>
  );
}

interface AuthState {
  required: boolean;
  configured?: boolean;
  user: { id: string; login: string; preferredName: string } | null;
}
function signIn(robot?: string) {
  sessionStorage.setItem("world-join-after-login", robot ?? "observe");
  location.assign("/auth/login");
}

export function LobbyApp() {
  const [auth, setAuth] = useState<AuthState | null>(null);
  const [nickname, setNickname] = useState(
    sessionStorage.getItem("world-display-name") ?? "",
  );
  const [hovered, setHovered] = useState<string | null>(null);
  const [joining, setJoining] = useState<string | null>(null);
  const nameDialog = useRef<HTMLDialogElement>(null);
  const nameInput = useRef<HTMLInputElement>(null);
  const [portraits, setPortraits] = useState<Record<string, string>>({});
  const [entry, setEntry] = useState<Entry | null>(null);
  const [entered, setEntered] = useState(
    sessionStorage.getItem("world-entered") === "yes",
  );
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const revision = useRef(0);
  const pending = useRef(false);
  const session = useWorldSession(auth?.required ? (entered && auth.user ? entry : null) : entry);
  const preview = useLobbyPreview(
    !entered || entry?.role === "observe" ? session : null,
  );
  useEffect(() => {
    let disposed = false;
    let retry: ReturnType<typeof setTimeout>;
    const open = async () => {
      try {
        const authResponse = await fetch("/api/auth", { signal: AbortSignal.timeout(8000) });
        const identity: AuthState = authResponse.status === 404
          ? { required: false, user: null }
          : authResponse.ok
          ? await authResponse.json()
          : (() => {
            throw new Error("Sign-in unavailable");
          })();
        if (disposed) return;
        setAuth(identity);
        const previous = sessionStorage.getItem("world-github-id");
        if (identity.required && previous !== identity.user?.id) {
          sessionStorage.removeItem("world-ticket");
          sessionStorage.removeItem("world-entered");
          setEntered(false);
          if (identity.user) sessionStorage.setItem("world-github-id", identity.user.id);
          else sessionStorage.removeItem("world-github-id");
        }
        if (identity.required && !identity.user) {
          const response = await fetch("/api/lobby", { signal: AbortSignal.timeout(8000) });
          if (!response.ok) throw new Error("World offline");
          const state = await response.json();
          if (!disposed) {
            setEntry({ ...state, token: "", role: "observe", robot: "world", runtime: "world" });
            setError("");
          }
          return;
        }
        const data = await request("create", sessionStorage.getItem("world-ticket") ?? undefined);
        if (disposed) return;
        sessionStorage.setItem("world-ticket", data.token);
        const afterLogin = sessionStorage.getItem("world-join-after-login");
        if (data.role !== "observe") {
          setEntered(true);
          sessionStorage.setItem("world-entered", "yes");
        } else if (afterLogin && identity.user) {
          sessionStorage.removeItem("world-join-after-login");
          if (afterLogin === "observe") {
            setEntered(true);
            sessionStorage.setItem("world-entered", "yes");
          } else {
            setNickname(
              identity.user.preferredName || sessionStorage.getItem("world-display-name") || "",
            );
            setJoining(afterLogin);
          }
        } else if (new URLSearchParams(location.search).has("host")) {
          setEntered(false);
          sessionStorage.removeItem("world-entered");
          setJoining("duck1");
        }
        setEntry(data);
        setError("");
      } catch {
        if (!disposed) {
          setError("The world is reconnecting. Trying again…");
          retry = setTimeout(open, 3000);
        }
      }
    };
    void open();
    return () => {
      disposed = true;
      clearTimeout(retry);
    };
  }, []);
  useEffect(() => {
    if (!entry) return;
    let disposed = false;
    const poll = setInterval(() => {
      if (pending.current) return;
      const version = revision.current;
      const update: Promise<Entry> = auth?.required && !auth.user
        ? fetch("/api/lobby").then(async (r) => {
          if (!r.ok) throw new Error("World offline");
          return {
            ...await r.json(),
            token: "",
            role: "observe",
            robot: "world",
            runtime: "world",
          };
        })
        : request("create", entry.token);
      update.then((data) => {
        if (disposed || version !== revision.current) return;
        sessionStorage.setItem("world-ticket", data.token);
        if (
          data.token !== entry.token ||
          (entry.role !== "observe" && data.role === "observe")
        ) {
          setEntered(false);
          sessionStorage.removeItem("world-entered");
          setError("Your session ended. Choose a duck or watch the world.");
        }
        setEntry(data);
      }).catch((error) => {
        if (disposed || version !== revision.current) return;
        if (auth?.required && error instanceof LobbyError && error.status === 401) {
          sessionStorage.removeItem("world-ticket");
          sessionStorage.removeItem("world-entered");
          setEntered(false);
          setJoining(null);
          setAuth({ ...auth, user: null });
          setError("Your sign-in expired. Sign in with GitHub to join again.");
        }
      });
    }, 3000);
    return () => {
      disposed = true;
      clearInterval(poll);
    };
  }, [entry?.token, entry?.role, auth]);
  useEffect(() => {
    const dialog = nameDialog.current;
    if (joining && dialog && !dialog.open) {
      dialog.showModal();
      nameInput.current?.focus();
      nameInput.current?.select();
    } else if (!joining && dialog?.open) dialog.close();
  }, [joining]);
  const choose = async (robot?: string, returnToLobby = false) => {
    if (auth?.required && !auth.user) {
      signIn(robot);
      return;
    }
    if (!entry || pending.current) return;
    pending.current = true;
    revision.current++;
    setBusy(true);
    setError("");
    try {
      const next = await request(
        robot ? "join" : "observe",
        entry.token,
        robot,
        robot ? nickname : undefined,
      );
      if (robot && next.displayName) {
        setNickname(next.displayName);
        sessionStorage.setItem("world-display-name", next.displayName);
      }
      setEntry(next);
      setJoining(null);
      setEntered(!returnToLobby);
      if (returnToLobby) sessionStorage.removeItem("world-entered");
      else sessionStorage.setItem("world-entered", "yes");
      if (new URLSearchParams(location.search).has("host")) {
        history.replaceState(null, "", location.pathname);
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : "Could not enter.");
    } finally {
      pending.current = false;
      setBusy(false);
    }
  };
  const joiningSlot = entry?.slots.find((slot) => slot.id === joining);
  const joiningUnavailable = !!joiningSlot?.occupied && !joiningSlot.mine;
  const occupied = entry?.slots.filter((s) => s.occupied).length ?? 0;
  return (
    <div
      className={styles.shell}
      data-testid="world-lobby"
      data-role={entry?.role ?? "loading"}
      data-robot={entry?.robot ?? ""}
    >
      <header className={styles.header}>
        <div className={styles.brand}>
          <span className={styles.brandIcon} aria-hidden="true">d</span> DIMENSIONAL
        </div>
        <div className={styles.capacity} data-testid="world-capacity">
          <i /> {entry ? `${occupied} / 6 ducks occupied` : "Connecting…"}
        </div>
        {auth?.user && (
          <div className={styles.actions}>
            <span>@{auth.user.login}</span>
            <button
              onClick={async () => {
                const response = await fetch("/auth/logout", { method: "POST" });
                if (response.ok) {
                  for (const key of ["world-ticket", "world-entered", "world-github-id", "world-join-after-login", "world-display-name"]) sessionStorage.removeItem(key);
                  location.reload();
                } else setError("Could not sign out. Please try again.");
              }}
            >
              Sign out
            </button>
          </div>
        )}
        {entered && entry && (
          <div className={styles.actions}>
            <span>
              {entry.role === "observe" ? "Spectator" : entry.displayName &&
                  entry.displayName !== playerName(entry.robot)
                ? `${entry.displayName} · ${playerName(entry.robot)}`
                : playerName(entry.robot)}
            </span>
            <button
              data-testid="leave-world"
              disabled={busy}
              onClick={() => choose(undefined, true)}
            >
              {entry.role === "observe" ? "Back to lobby" : "Leave duck"}{" "}
              <span aria-hidden="true">↗</span>
            </button>
          </div>
        )}
      </header>
      {error && !joining && <div role="alert" className={styles.error}>{error}</div>}
      {entered && entry
        ? (
          <main className={styles.world}>
            {session
              ? <World entry={entry} session={session} preview={preview} />
              : (
                <div className={styles.loading} role="status">
                  Connecting to the world…
                </div>
              )}
          </main>
        )
        : (
          <main className={styles.landing}>
            <div className={styles.intro}>
              <h1>
                Welcome to a <span>new DIMENSION</span>
              </h1>
              <p>
                Each microduck has its own context, the world is a shared MuJoCo environment.
              </p>
            </div>
            <HoverPreview
              asset={preview.duck}
              active={hovered}
              onPortraits={setPortraits}
            />
            <div className={styles.teamHeading}>
              <div className={styles.teamSummary} aria-label="Team availability">
                {teams.map((team) => (
                  <span key={team} data-team={team}>
                    <b>{team === "red" ? "Red team" : "Blue team"}</b>
                    {robotIds.filter((id) =>
                      roster[id].team === team &&
                      !entry?.slots.find((s) => s.id === id)?.occupied
                    ).length} / 3 available
                  </span>
                ))}
              </div>
              <p>
                {auth?.required && !auth.user
                  ? "Sign in with GitHub to play or watch. Choose a duck to continue."
                  : "Choose your duck. Meet your team beside the midfield touchline."}
              </p>
            </div>
            <div className={styles.choices}>
              {robotIds.map(
                (id) => {
                  const player = roster[id];
                  const color = duckColors[id];
                  const slot = entry?.slots.find((s) => s.id === id);
                  const unavailable = !!slot?.occupied && !slot.mine;
                  const availability = !entry
                    ? "Connecting"
                    : slot?.mine
                    ? "Your duck"
                    : unavailable
                    ? slot.connected ? "In use" : `Reconnecting · ${slot.reconnectSeconds}s`
                    : "Available";
                  return (
                    <article
                      key={id}
                      className={`${styles.character} ${unavailable ? styles.unavailable : ""}`}
                      style={{ "--duck": color } as CSSProperties}
                      data-testid={`card-${id}`}
                      onPointerEnter={(event) => {
                        if (event.pointerType !== "touch") setHovered(id);
                      }}
                      onPointerLeave={(event) => {
                        if (event.pointerType !== "touch") setHovered(null);
                      }}
                      onFocusCapture={(event) => {
                        if (
                          (event.target as HTMLElement).matches(
                            ":focus-visible",
                          )
                        ) setHovered(id);
                      }}
                      onBlurCapture={(event) => {
                        if (
                          !event.currentTarget.contains(event.relatedTarget)
                        ) setHovered(null);
                      }}
                    >
                      <div className={styles.cardTop}>
                        <span className={styles.player}>
                          {player.team === "red" ? "R" : "B"}0{player.number}
                        </span>
                        <span className={styles.availability}>
                          <i />
                          {availability}
                        </span>
                      </div>
                      <div className={styles.portrait} data-preview={id}>
                        <span className={styles.orbit} aria-hidden="true" />
                        {portraits[id]
                          ? (
                            <img
                              src={portraits[id]}
                              alt={`${player.name} Microduck`}
                            />
                          )
                          : (
                            <span className={styles.previewFallback}>
                              {player.number}
                            </span>
                          )}
                        <button
                          type="button"
                          className={styles.previewButton}
                          aria-label={`${
                            hovered === id ? "Stop" : "Preview"
                          } ${player.name} walking`}
                          aria-pressed={hovered === id}
                          onClick={() => setHovered(hovered === id ? null : id)}
                        >
                          {hovered === id ? "Stop preview" : "Preview walk"}
                        </button>
                      </div>
                      <div className={styles.cardBody}>
                        <h2>{player.name}</h2>
                        {slot?.displayName && <p className={styles.occupant}>{slot.displayName}</p>}
                        <span className={styles.knowledge}>
                          PITCH + LOCKER ROOMS KNOWN
                        </span>
                        <button
                          data-testid={`choose-${id}`}
                          disabled={!entry || busy || unavailable}
                          onClick={() => {
                            setHovered(null);
                            setError("");
                            if (auth?.required && !auth.user) signIn(id);
                            else setJoining(id);
                          }}
                        >
                          {slot?.mine
                            ? `Continue as ${player.name}`
                            : unavailable
                            ? "Duck is occupied"
                            : `Play as ${player.name}`}
                          <span aria-hidden="true">↗</span>
                        </button>
                      </div>
                    </article>
                  );
                },
              )}
            </div>
            <aside className={styles.spectator} aria-label="Join as a viewer">
              <div className={styles.roomPreview}>
                <RoomPreview model={preview.model} snapshot={preview.snapshot} />
              </div>
              <div>
                <h2>Just watching?</h2>
                <p>Explore the club with a free camera and live score. No duck needed.</p>
              </div>
              <button
                data-testid="observe-world"
                disabled={!entry || busy}
                onClick={() => choose()}
              >
                Watch the match <span aria-hidden="true">↗</span>
              </button>
            </aside>
            <footer className={styles.footer}>
              <span>
                <kbd>W</kbd>
                <kbd>A</kbd>
                <kbd>S</kbd>
                <kbd>D</kbd> to walk <b>·</b> Talk to your agent to explore
              </span>
              <span>
                Your discoveries stay private. A new player starts fresh.
              </span>
            </footer>
          </main>
        )}
      <dialog
        ref={nameDialog}
        className={styles.nameDialog}
        aria-labelledby="join-title"
        onCancel={(event) => {
          if (busy) event.preventDefault();
          else setJoining(null);
        }}
      >
        <form
          onSubmit={(event) => {
            event.preventDefault();
            if (joining && !joiningUnavailable) void choose(joining);
          }}
        >
          <span className={styles.eyebrow}>MEET YOUR TEAM</span>
          <h2 id="join-title">Join as {joining ? playerName(joining) : "a player"}</h2>
          <p>What should the other players call you?</p>
          <label htmlFor="player-name">Your player name</label>
          <input
            ref={nameInput}
            id="player-name"
            data-testid="player-name"
            type="text"
            maxLength={24}
            autoComplete="nickname"
            placeholder="e.g. Tule"
            value={nickname}
            disabled={busy}
            aria-describedby="player-name-hint"
            onChange={(event) => {
              setNickname(event.target.value);
              sessionStorage.setItem("world-display-name", event.target.value);
            }}
          />
          <p id="player-name-hint" className={styles.nameHint}>
            Shown above your duck. Leave blank to use its team number.
          </p>
          {(error || joiningUnavailable) && (
            <p role="alert" className={styles.joinError}>
              {joiningUnavailable ? "This duck was just taken. Choose another duck." : error}
            </p>
          )}
          <div className={styles.dialogActions}>
            <button type="button" disabled={busy} onClick={() => setJoining(null)}>Cancel</button>
            <button
              data-testid="confirm-join"
              type="submit"
              disabled={busy || !entry || joiningUnavailable}
            >
              {busy ? "Connecting…" : `Join ${joining ? playerName(joining) : "duck"}`}
            </button>
          </div>
        </form>
      </dialog>
    </div>
  );
}
