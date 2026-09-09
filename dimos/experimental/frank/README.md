# Frank

Frank turns a Unitree Go2 into a social host: it recognizes visitors, chats with them
through a phone app, speaks aloud, and uses robot skills to move and find people.

## Setup

Run all commands from the repository root. You need a network-connected Go2
and an audio output device.

```bash
uv sync --extra all
npm i -g @earendil-works/pi-coding-agent
cp -n dimos/experimental/frank/.env.example dimos/experimental/frank/.env
```

Configure the two environment files:

- **`dimos/experimental/frank/.env`:** fill in `ELEVENLABS_API_KEY`,
  `ELEVENLABS_VOICE_ID`, and `CEREBRAS_API_KEY`. Set `FRANK_ROBOT_IP` to your
  Go2's address (default: `10.0.0.79`).
- **Repository-root `.env`:** set `DIMOS_UNITREE_AES_128_KEY` for your Go2.

Select your speaker as the system's default audio output, then test it:

```bash
uv run python dimos/experimental/frank/tools/speak.py "Hello, I'm Frank."
```

## Start

**Starting the stack makes the robot stand up. Motion is enabled by default.**

```bash
uv run python dimos/experimental/frank/up.py
```

This starts the robot stack, phone server, face watcher, and Frank's agent loop,
then opens the operator dashboard. Existing services are reused.
The face model downloads on first use; visitor selfies are enrolled automatically.

- **Dashboard:** http://127.0.0.1:7790/ops (local access only).
- **Chat app:** http://localhost:7790 on the computer running Frank.
- **Rerun viewer:** add `--rerun` at startup; close it when finished to limit memory use.

## Phone access (optional)

Local use needs no Tailscale. For visitors' phones, use a reachable HTTPS address;
`localhost` on a phone refers to the phone itself. Tailscale Funnel provides this
address, and `up.py` starts it automatically when Tailscale is installed.
Enable Funnel on your tailnet and allow your user to manage it with
`sudo tailscale set --operator=$USER`. `FRANK_FUNNEL_URL` overrides the advertised URL.

To generate a QR page for visitors, enter the HTTPS URL printed at startup:

```bash
read -r -p "Phone app HTTPS URL: " FRANK_PHONE_URL
uv run python dimos/experimental/frank/tools/qr.py "$FRANK_PHONE_URL"
```

This writes `qr.png`. Add `--print` to send it to your default printer.

## Operate and stop

Use the dashboard to restart Frank after editing his guidance, view turn logs,
or toggle motion. **Halt** cancels movement and disables motion until re-enabled.
For an emergency, use the Unitree remote first; the CLI stop command is:

```bash
uv run python dimos/experimental/frank/tools/robot.py stop
```

Ctrl-C stops the foreground agent loop; the other services keep running.

```bash
uv run python dimos/experimental/frank/up.py status
uv run python dimos/experimental/frank/up.py down
```

Add `--wipe` to `down` to delete visitor data, including chats and selfies.
Recordings are retained separately in `recordings/<run-id>/memory.db`; remove old
recordings manually. Recording is enabled for Frank's memory and uses roughly 6 GB/hour.

## Reference

- [Agent guidance](/dimos/experimental/frank/SKILL.md) and [persona](/dimos/experimental/frank/persona.md): Frank's runtime instructions.
- [Phone app](/dimos/experimental/frank/app/README.md) and [API](/dimos/experimental/frank/app/API.md): server setup and integration.
- [Memory API](/dimos/experimental/frank/docs/memory_api.md): recording and recall details.
- `up.py`: service orchestration; `loop.py`: agent loop; `tools/`: operator CLIs;
  `pi/`: agent extensions.

Frank uses `cerebras/gemma-4-31b` by default; override it with `up.py --model` or
`FRANK_MODEL`. Optional robot vision settings are in `.env.example` and require
restarting the robot stack.
