# Issue tracking with Linear

## Workspace

DimOS work is tracked in the **DIM** team in Linear:

<https://linear.app/dimensional/team/DIM>

Access Linear through the configured Linear MCP. Do not assume that a local
copy, an unconfigured client, or a direct API call is an alternative source of
truth.

## Confirmation policy

User confirmation is required immediately before **every** Linear edit. This
includes, without limitation:

- creating an issue;
- changing any issue field, including title, description, assignee, priority,
  project, or due date;
- adding, removing, or changing labels;
- posting comments;
- changing state or making any other state transition; and
- adding, removing, or changing links.

Reading Linear is not an edit. Before an edit, state exactly what will change
and wait for explicit user confirmation. One confirmation does not authorize
later edits, even when they concern the same issue or change.

## Linking convention

Keep the work chain navigable:

```text
Linear issue  <->  openspec/changes/<change-id>  <->  pull request
```

Use the OpenSpec change ID as the stable identifier in the relationship. Link
the Linear issue to the relevant OpenSpec change and link the pull request to
both when the tools support those links. If a link must be created or changed,
it is a Linear edit and requires confirmation under the policy above.

## Source of truth and workflow

Linear is the intake and tracking system. It records requests, ownership,
status, discussion, and delivery progress. OpenSpec is the source of truth for
the behavioral change, its design, and its implementation tasks. The pull
request is the review and delivery vehicle.

Use this sequence:

1. Capture or find the Linear issue in the DIM team.
2. Create or update `openspec/changes/<change-id>/` for the proposed behavior,
   design, and tasks.
3. Implement the tasks and keep the OpenSpec change current.
4. Open the pull request and connect it to the issue and OpenSpec change.
5. Reflect progress in Linear only after confirming each requested edit.

Do not use a Linear description, comment, or state as a substitute for an
OpenSpec requirement, design decision, or task. If Linear and OpenSpec
disagree about behavior, treat OpenSpec as authoritative and surface the
discrepancy to the user rather than silently choosing a version.
