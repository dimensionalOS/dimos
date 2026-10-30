import assert from "node:assert/strict";
import { test } from "node:test";
import { ChordError } from "@earendil-works/chord";
import type { ConversationId } from "@earendil-works/pi-durable";
import { Feedback } from "./feedback.ts";
import { newUserRequest } from "../../shared/protocol/actions/request.ts";
import { done, refused } from "../../shared/protocol/outcome.ts";
import { actionLine, pendingSends } from "../../shared/view-models/actions.ts";
import { RefusalError } from "../../sdk/failures.ts";

test("feedback keeps pending sends visible and reports server refusals, unknown outcomes and terminal closure", async () => {
  let close: (error: Error) => void = () => {};
  const ended = new Promise<never>((_, reject) => {
    close = reject;
  });
  const feedback = new Feedback(ended);
  let finish: () => void = () => {};
  const pending = new Promise<ReturnType<typeof done<null>>>((resolve) => {
    finish = () => resolve(done(null));
  });
  const request = newUserRequest();
  const conversation = 1 as ConversationId;
  const observed = feedback.observe("send", pending, { conversation, sending: { request, text: "hello" } });
  assert.deepEqual(pendingSends(feedback.value, conversation), [{ request, text: "hello" }]);
  assert.equal(actionLine(feedback.value[0]), "⟳ send…");
  finish();
  await observed;
  assert.deepEqual(pendingSends(feedback.value, conversation), []);
  await feedback.observe("change model", Promise.resolve(refused({ code: "running", conversation })));
  const rejected = feedback.value.at(-1);
  assert.ok(rejected);
  assert.match(actionLine(rejected), /press Esc to stop/);
  await feedback.observe("restart module", Promise.reject(new ChordError("unavailable", "reply lost")));
  assert.equal(feedback.value.at(-1)?.status, "unknown");
  const afterClose = feedback.observe("send", new Promise<never>(() => {}));
  close(new RefusalError({ code: "stopping" }));
  assert.deepEqual(await afterClose, refused({ code: "stopping" }));
  const stopped = feedback.value.at(-1);
  assert.ok(stopped);
  assert.equal(actionLine(stopped), "✖ send: The gateway is stopping.");
});
