/**
 * @module
 *
 * **DimSim** — 3D simulation environment for the
 * [dimos](https://github.com/dimensionalOS/dimos) robotics stack.
 *
 * Provides a browser-based Three.js + Rapier simulator with LCM transport,
 * sensor publishing (RGB, depth, LiDAR, odometry), and an eval harness for
 * automated testing of navigation and perception pipelines.
 *
 * ## Install
 *
 * ```sh
 * deno install -gAf --unstable-net jsr:@antim/dimsim
 * ```
 *
 * ## Setup
 *
 * Download core assets (~22 MB) and install a scene:
 *
 * ```sh
 * dimsim setup
 * dimsim scene install apt
 * ```
 *
 * ## Run
 *
 * Start the dev server and open the URL it prints:
 *
 * ```sh
 * dimsim dev --scene apt
 * ```
 *
 * Run headless evals in CI:
 *
 * ```sh
 * dimsim eval --headless --env apt --workflow reach-vase
 * ```
 *
 * ## Programmatic API
 *
 * ```ts
 * import { startBridgeServer } from "@antim/dimsim";
 *
 * startBridgeServer({ port: 8090, distDir: "./dist", scene: "apt" });
 * ```
 */

/** Start the WebSocket bridge server that relays LCM packets between the browser and external agents. */
export { startBridgeServer } from "./bridge/server.ts";

/** Launch a single headless Chromium page pointed at the sim. */
export { launchHeadless } from "./headless/launcher.ts";

/** Launch multiple headless pages for parallel eval workflows. */
export { launchMultiPage } from "./headless/launcher.ts";

/** Run eval workflows sequentially against a connected browser. */
export { runEvals } from "./eval/runner.ts";

/** Run eval workflows distributed across multiple browser pages. */
export { runEvalsMultiPage } from "./eval/runner.ts";

/** Collect workflow definitions from the manifest, optionally filtered by env/workflow name. */
export { collectWorkflows } from "./eval/runner.ts";

/** Convert eval results to JUnit XML format for CI reporting. */
export { toJunitXml } from "./eval/runner.ts";

/** Download and extract core DimSim assets to ~/.dimsim/. */
export { setup } from "./setup.ts";

/** Download and install a scene by name from the registry. */
export { sceneInstall } from "./setup.ts";

/** List installed and available scenes. */
export { sceneList } from "./setup.ts";

/** Remove a locally installed scene. */
export { sceneRemove } from "./setup.ts";

/** Get the DimSim home directory path (~/.dimsim or DIMSIM_HOME). */
export { getDimsimHome } from "./setup.ts";

/** Get the path to the dist directory containing built frontend assets. */
export { getDistDir } from "./setup.ts";
