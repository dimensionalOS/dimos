# Proposed live agent acceptance test

Destination: OpenAI Responses API at `https://api.openai.com/v1/responses`.
Configured model: `gpt-5.6-luna` (the existing McpClient default).
Authentication: the existing host `OPENAI_API_KEY`; its value is never logged.

Five bounded trials, each with this request:

> Reset the simulated scene, then put bottle_1 and bottle_4 in the bin using the appropriate arm for each. Return both arms home and verify both are in the bin. Do not retry a failed pick/place during this acceptance trial; report the failure.

The model receives the bimanual system prompt from
`dimos/robot/manipulators/common/agent_prompts.py`, JSON schemas and descriptions
for the registered manipulation and simulation skills, and the trial's text
conversation. Tool results contain simulated bottle positions, bin bounds,
joint/end-effector state, grasp/placement status, and error messages.

This blueprint supplies privileged numeric simulation observations. Its camera
streams feed the local viewer; these skills do not send camera images to the
model. Repository files, recordings, dataset frames, environment variables,
and credential values are not included in the model input.

The model's tool calls execute through the local MCP server at port 19990 and
can move only this task's MuJoCo rig. The harness uses a separate local transport
bus and checks physical containment after each trial. Raw model request/response
bodies and trial outcomes are saved locally under `/tmp/openyam-agent-traces`
and `/tmp/openyam-agent-acceptance.jsonl`. The harness times out a trial after
240 seconds and shuts down its own stack.

The diagnostic ACT policy has already passed real-process preflight, motion,
stop, and reset checks. The scripted classical sequence has passed five of five
bimanual physical-containment trials. This proposed test adds actual model
selection and sequencing of those skills.
