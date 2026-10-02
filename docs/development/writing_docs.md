# Writing Docs

1. Where to put your docs:
    - If it only matters to people who contribute to dimos (like this doc), put them in `docs/development`
    - Otherwise put them in `docs/usage`
    - The web stack (`docs/web`) is the exception: its user, hosting and contributor pages live together, because one reader moves between them
2. Diagrams are [pikchr](https://pikchr.org/home/doc/trunk/doc/userman.md) blocks, rendered to SVG with `bin/gen-diagrams docs/path/page.md` (it needs the `pikchr` binary, so run it as `nix shell nixpkgs#pikchr -c uv run bin/gen-diagrams docs/path/page.md` when the binary is not installed). The fence syntax is in [Code Blocks](/docs/coding-agents/docs/index.md#pikchr). md-babel writes the image line as `![output](...)`, so put the descriptive alt text back after regenerating. The site does not render mermaid.
3. Use [md-babel-py](https://github.com/leshy/md-babel-py/) (`md-babel-py run thing.md`) to make sure your code examples work.
