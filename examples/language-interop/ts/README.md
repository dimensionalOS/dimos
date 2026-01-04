# TypeScript Robot Control Examples

Subscribes to `/odom` and publishes velocity commands to `/cmd_vel`.

## CLI Example

```bash
deno task start
```

## Web Example

Browser-based control with WebSocket bridge:

```bash
cd web
deno run --allow-net --allow-read --unstable-net server.ts
```

Open http://localhost:8080 in your browser.

Features:
- Real-time pose display
- Arrow keys / WASD for control
- Click buttons to send twist commands

The browser imports `@dimos/msgs` via [esm.sh](https://esm.sh) and encodes/decodes LCM packets directly - the server just forwards raw binary between WebSocket and UDP multicast.

## Dependencies

Main documentation for TS interop:

- [@dimos/lcm](https://jsr.io/@dimos/lcm)
- [@dimos/msgs](https://jsr.io/@dimos/msgs)
