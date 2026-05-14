/**
 * Eval Builder — generate workflow JSON from validated inputs.
 *
 * An eval = rubric + target + timeout. This module validates the target
 * exists in the scene, generates the workflow JSON, and updates the manifest.
 */

import { loadSceneIndex, findObject, findAllObjects, suggestObjects } from "./scene-index.ts";

export interface BuildEvalOptions {
  scenePath: string;
  sceneName: string;
  target: string;
  threshold?: number;
  timeout?: number;
  task?: string;
  name?: string;
  env?: string;
  evalsDir: string;
}

export interface BuildResult {
  filePath: string;
  workflowName: string;
  task: string;
  targetTitle: string;
  targetPosition: { x: number; y: number; z: number };
  threshold: number;
  timeout: number;
  env: string;
}

function slugify(s: string): string {
  return s.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "");
}

export function buildEval(opts: BuildEvalOptions): BuildResult {
  const index = loadSceneIndex(opts.scenePath, opts.sceneName);

  // Validate target exists in scene
  const match = findObject(opts.target, index);
  if (!match) {
    const suggestions = suggestObjects(opts.target, index);
    let msg = `No asset matching "${opts.target}" in scene "${opts.sceneName}".`;
    if (suggestions.length > 0) {
      msg += `\nSimilar: ${suggestions.join(", ")}`;
    }
    msg += `\nHint: dimsim list objects --scene ${opts.sceneName}`;
    throw new Error(msg);
  }

  // Warn about duplicates
  const allMatches = findAllObjects(opts.target, index);
  if (allMatches.length > 1) {
    console.warn(
      `[build] Warning: "${opts.target}" matches ${allMatches.length} objects. ` +
      `Using first: "${match.title}" at (${match.position.x}, ${match.position.y}, ${match.position.z})`
    );
  }

  const threshold = opts.threshold ?? 2.0;
  const timeout = opts.timeout ?? 60;
  const env = opts.env || opts.sceneName;
  const workflowName = opts.name || slugify(opts.target);
  const task = opts.task || `Go to the ${match.title}`;

  const workflow = {
    name: workflowName,
    environment: env,
    task,
    startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
    timeoutSec: timeout,
    successCriteria: {
      objectDistance: {
        object: "agent",
        target: opts.target,
        thresholdM: threshold,
      },
    },
  };

  // Write workflow JSON
  const envDir = `${opts.evalsDir}/${env}`;
  try { Deno.mkdirSync(envDir, { recursive: true }); } catch { /* exists */ }
  const filePath = `${envDir}/${workflowName}.json`;
  Deno.writeTextFileSync(filePath, JSON.stringify(workflow, null, 2) + "\n");

  // Update manifest
  updateManifest(`${opts.evalsDir}/manifest.json`, env, opts.sceneName, workflowName);

  return {
    filePath,
    workflowName,
    task,
    targetTitle: match.title,
    targetPosition: match.position,
    threshold,
    timeout,
    env,
  };
}

function updateManifest(manifestPath: string, env: string, scene: string, workflowName: string): void {
  let manifest: { version: string; environments: { name: string; scene: string; workflows: string[] }[] };

  try {
    manifest = JSON.parse(Deno.readTextFileSync(manifestPath));
  } catch {
    manifest = { version: "1.0", environments: [] };
  }

  let envEntry = manifest.environments.find((e) => e.name === env);
  if (!envEntry) {
    envEntry = { name: env, scene, workflows: [] };
    manifest.environments.push(envEntry);
  }

  if (!envEntry.workflows.includes(workflowName)) {
    envEntry.workflows.push(workflowName);
  }

  Deno.writeTextFileSync(manifestPath, JSON.stringify(manifest, null, 2) + "\n");
}
