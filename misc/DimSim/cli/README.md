# DimSim

3D simulation environment for the [dimos](https://github.com/dimensionalOS/dimos) robotics stack.

Browser-based Three.js + Rapier simulator with LCM transport, sensor publishing (RGB, depth, LiDAR, odometry), and an eval harness for automated testing of navigation and perception pipelines.

## Install

```sh
deno install -gAf --unstable-net jsr:@antim/dimsim
```

## Setup

Download core assets (~22 MB) and install a scene:

```sh
dimsim setup
dimsim scene install apt
```

## Run

Start the dev server and open the URL it prints:

```sh
dimsim dev --scene apt
```

Run headless evals in CI:

```sh
dimsim eval --headless --env apt --workflow reach-vase
```

## Programmatic API

```ts
import { startBridgeServer } from "@antim/dimsim";

startBridgeServer({ port: 8090, distDir: "./dist", scene: "apt" });
```

## Commands

| Command | Description |
|---------|-------------|
| `dimsim setup` | Download core assets |
| `dimsim scene install <name>` | Install a scene |
| `dimsim scene list` | List available and installed scenes |
| `dimsim scene remove <name>` | Remove a scene |
| `dimsim dev [--scene <name>]` | Dev server (open browser manually) |
| `dimsim eval --headless` | Run eval workflows in CI |
| `dimsim agent` | Launch dimos Python agent |

## License

MIT
