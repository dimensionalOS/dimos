# Goldie Webapp — briefing

The phone-facing control app for the Unitree Go2, **built for low-vision users**:
hold-to-speak voice commands, a manual joystick, live agent responses, and
**on-device text-to-speech** of what the robot says. It is the only piece in
`/webapp`; it talks to a DimOS backend over HTTP + SSE (and Socket.IO for the
joystick). It never touches the robot directly.

> This replaced an earlier App-Router stub. That stub is preserved verbatim in
> [`SCAFFOLD-REFERENCE.md`](./SCAFFOLD-REFERENCE.md) because it was wired to the
> monorepo backend's contract — see **Backend contracts** below.

## Quick start

```bash
cd webapp
npm install
cp .env.example .env.local   # then edit (see Environment)
npm run dev                  # http://localhost:3000
```

With **no** `.env.local`, it runs fully against a built-in **mock** (`/api/mock`)
— so the UI works with no backend at all. For a real iPhone, expose `:3000`
through a tunnel (ngrok/Tailscale) and open it in Safari.

## Stack

Next.js 16 (**Pages Router**, `src/pages`), React 19, TypeScript (strict),
**Tailwind v4** (CSS-first `@theme` in `src/styles/globals.css`), Vitest.
Self-contained — it does not depend on the rest of the monorepo at build time.

## Structure

```
src/
  pages/
    index.tsx              # the one screen: orchestrates everything
    _app.tsx, _document.tsx# viewport + iOS/PWA meta
    api/mock/[...path].ts  # mock backend (SSE + endpoints) for offline dev
    api/log.ts             # dev: prints speech/tts events to the `npm run dev` terminal
  components/              # Header, ModeToggle, VoiceButton, StatusCard,
                           # QuickActions, InterruptButton, Joystick, Voice/ManualPanel
  hooks/                   # useStt, useAgentFeed, useStatus, useTeleop
  lib/                     # dimos.ts (API client), stt.ts, speech.ts (TTS),
                           # agentMessage.ts, joystick.ts, types.ts
public/                    # manifest.json + icons (Add-to-Home-Screen / PWA)
```

## Backend contracts ⚠️ (read before integrating)

There are **two** DimOS contracts in play. Goldie currently targets the first;
the monorepo backend uses the second.

| | Goldie targets now | Monorepo backend (Tailscale :8443) |
|---|---|---|
| SSE stream key | `agent_responses` (plain text) | `agent_state` (structured JSON) |
| Auth | `ngrok-skip-browser-warning` header | `Authorization: Bearer` + `?token=` on SSE |
| State shape | text lines | `{ phase, current_skill, last_observation, last_narration, awaiting_user, intent }` |
| Input tags | `<user_speech>` | `<user_speech>` / `<user_reply>` (when `awaiting_user`) / `<user_command>` |

**Common to both:** `POST /submit_query` as **multipart form-data**, field
`query` (never JSON). Voice transcripts are wrapped in `<user_speech>…</user_speech>`.

**To run Goldie against the monorepo backend** (see `SCAFFOLD-REFERENCE.md` for
the working example): in `src/lib/dimos.ts` + `src/hooks/useAgentFeed.ts`, switch
the stream key to `agent_state`, send the token (Bearer + `?token=`), map
`last_narration`/`awaiting_user` into the feed, add the `awaiting_user →
<user_reply>` flow, and speak `last_narration`/`awaiting_user` via TTS
(`src/lib/speech.ts`). The mock currently emulates the `agent_responses` shape.

## Environment

| Var | Purpose |
|---|---|
| `NEXT_PUBLIC_DIMOS_API` | Voice/agent API + SSE base. Unset → built-in mock. |
| `NEXT_PUBLIC_DIMOS_VIS` | Socket.IO vis/teleop server (joystick, `move_command`). |
| `NEXT_PUBLIC_DIMOS_TOKEN` | Bearer token (monorepo backend). |
| `NEXT_PUBLIC_STT` | `webspeech` (default, on-device) or `upload` (MediaRecorder → `/upload_audio`). |

## Features

- **Voice mode** — hold-to-speak. STT is the on-device Web Speech API by default
  (`lib/stt.ts`, behind a swappable provider; an `upload` provider posts
  `audio/mp4` to `/upload_audio` for server-side Whisper).
- **Manual mode** — analog joystick → Socket.IO `move_command` Twist on the vis
  server at ~15 Hz, zero-twist on release (`hooks/useTeleop.ts`, `lib/joystick.ts`).
- **Agent feed** — classified, de-duplicated, noise-filtered messages
  (`lib/agentMessage.ts`, `hooks/useAgentFeed.ts`).
- **Text-to-speech** — speaks agent replies via browser `SpeechSynthesis`
  (`lib/speech.ts`), on by default, with a header mute toggle.
- **PWA** — manifest + apple-touch-icon + iOS meta (Add-to-Home-Screen).

## iOS Safari caveats

- `MediaRecorder` needs `audio/mp4` (not webm); `getUserMedia` needs HTTPS + a tap.
- `EventSource` can't set headers → we use `@microsoft/fetch-event-source`.
- **TTS limitation (open issue):** iOS only allows `speechSynthesis` inside a user
  gesture; speech triggered by an incoming SSE message is silently dropped (works
  on desktop). We unlock on the talk-button tap, but auto-speaking replies is
  unreliable on iPhone. Reliable fallback for blind users: **VoiceOver** reads the
  `aria-live` response feed automatically.

## Accessibility

`aria-live` on the response feed, `aria-label`/`aria-pressed` on the talk button
and speech toggle. TODO: spoken cues for state changes (listening/sent/connected),
logical focus order, and resolving the iOS auto-TTS issue above.

## Testing

```bash
npm run test     # vitest: payload wrapping, message classification, joystick math
npm run build    # production build (also type-checks)
```

## Known gaps / deferred

- **Quick actions** (Sit/Jump/Lie down → `/unitree/command`) and **Interrupt**
  (`/interrupt`) are not on the verified backend — wired to the mock, route TBD.
- iOS auto-TTS (above).
- Wiring to the monorepo `agent_state` backend (above).
