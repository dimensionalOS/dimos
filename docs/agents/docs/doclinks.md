When writing or editing markdown documentation, use `doclinks` tool to resolve file references.

Full documentation if needed: [`utils/docs/doclinks.md`](/dimos/utils/docs/doclinks.md)

## Syntax

<!-- doclinks-ignore-start -->
| Pattern     | Example                                             |
|-------------|-----------------------------------------------------|
| Code file   | `[`service/spec.py`]()` → resolves path             |
| With symbol | `Configurable` in `[`spec.py`]()` → adds `#L<line>` |
| Doc link    | `[Configuration](.md)` → resolves to doc            |
<!-- doclinks-ignore-end -->

## Usage

```bash
doclinks docs/guide.md   # single file
doclinks docs/           # directory
doclinks --dry-run ...   # preview only
```
