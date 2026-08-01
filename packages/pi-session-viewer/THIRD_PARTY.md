# Frontend provenance

This viewer owns its application and component source. It does not run a
component generator during ordinary builds.

The message, reasoning, and tool presentation patterns in
`src/components/DisplayParts.tsx` and `src/components/EntryCard.tsx` were
adapted from the Vercel AI Elements `ai-elements@1.9.0` upstream release tag,
licensed under Apache-2.0:

- https://github.com/vercel/ai-elements/releases/tag/ai-elements%401.9.0
- upstream component families: `message`, `reasoning`, and `tool`
- local changes: replace AI SDK types with immutable Pi view-model types;
  remove live-chat hooks, actions, status polling, copy controls, and runtime
  dependencies; use Base UI collapsibles and the local design system.

Structural interactions use
[Base UI 1.6.0](https://github.com/mui/base-ui), licensed under MIT. Markdown
rendering uses [Streamdown 2.5.0](https://github.com/vercel/streamdown),
licensed under Apache-2.0. Icons use
[Lucide 1.26.0](https://github.com/lucide-icons/lucide), licensed under ISC.

Exact runtime and development dependencies are recorded in `package.json` and
`package-lock.json`. Future component updates require an explicit source diff
and provenance update.
