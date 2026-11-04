import type { Extension } from "@earendil-works/pi-durable";
import type { JobTask } from "#dimos/agents/harness/jobs/task.ts";
import { defineExtension } from "@earendil-works/pi-durable";

/** Registers the job task kind, so pending job tasks resume when the gateway starts. The job tool is in the tools extension's table. */
export function jobsExtension(task: JobTask): Extension {
  return defineExtension({ name: "dimos-jobs", tasks: [task] });
}
