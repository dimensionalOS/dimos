# Writing Docs

Note: as of the DimOS beta, not all existing docs conform to this guide, but newly added docs should.

## Need-to-know Things

1. Where to put your docs.
    - Some docs are under `docs/` (like this one) but others are stored in the actual codebase, like `dimos/robot/drone/README.md`.
    - If your docs have code examples and are somewhere under `docs/`, those code examples must be executable. See [codeblocks guide](/docs/development/writing_docs/codeblocks.md) for details and instructions on how to execute your code examples.
    - If your docs nicely *introduce* a new API, or they are a tutorial, then put them in `docs/concepts/` (even if they are about a specific API).
    - If the docs are highly technical or exhaustive there are a three options:
        - If your docs are about a user-facing API (ex: the reader can follow your instructions without cloning dimos) then put them in `docs/api/`.
        - Otherwise (if the reader is modifying their own copy of the dimos codebase) then your docs have two options:
            1. You can choose to store your docs next to relevant python files (ex: `dimos/robot/drone/README.md`), and we are less strict about the contents (code examples don't need to be executable) **BUT**, you need to edit something in `docs/development/` or `docs/api/` to add a reference/link to those docs (don't create "dangling" documentation).
            2. Alternatively, you can put your docs in `docs/development/`. Code examples there should be executable.
2. Even if you know how to link to other docs, read our [how we do doc linking guide](/docs/development/writing_docs/doclinks.md).
3. Even if you know how to create diagrams on your own, read our [how we do diagrams guide](/docs/development/writing_docs/diagram_practices.md).
