import { type AgentOutputEvent, parseAgentOutputLine } from "./agent-output.ts";

function assertEquals(actual: unknown, expected: unknown): void {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a !== e) throw new Error(`expected ${e}, got ${a}`);
}

Deno.test("agent output parser accepts ready and typed output events", () => {
  assertEquals(parseAgentOutputLine('{"type":"ready"}'), "ready");
  assertEquals(
    parseAgentOutputLine(
      '{"type":"agent_output","text":"FOUND_BATHTUB",' +
        '"hasToolCalls":false,"timestampMs":1234}',
    ),
    {
      type: "agent_output",
      text: "FOUND_BATHTUB",
      hasToolCalls: false,
      timestampMs: 1234,
    } satisfies AgentOutputEvent,
  );
});

Deno.test("agent output parser rejects malformed and incomplete lines", () => {
  assertEquals(parseAgentOutputLine("not json"), null);
  assertEquals(
    parseAgentOutputLine(
      '{"type":"agent_output","text":"FOUND_BATHTUB",' +
        '"hasToolCalls":"false","timestampMs":1234}',
    ),
    null,
  );
  assertEquals(
    parseAgentOutputLine(
      '{"type":"agent_output","text":"FOUND_BATHTUB",' +
        '"hasToolCalls":false,"timestampMs":null}',
    ),
    null,
  );
});
