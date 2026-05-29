When writing or editing markdown documentation, use `doclinks` tool to resolve file references.

Full documentation if needed: [`utils/docs/doclinks.md`](/dimos/utils/docs/doclinks.md)

## Syntax


| Pattern     | Example                                                                 |
|-------------|-------------------------------------------------------------------------|
| Code file   | `[`service/spec.py`](/dimos/protocol/service/spec.py#L29)` → resolves path |
| With symbol | `Configurable` in `[`service/spec.py`](/dimos/protocol/service/spec.py#L25)` → adds `#L<line>` |
| Doc link    | `[Configuration](/docs/usage/configuration.md)` → resolves to doc       |


## Usage

```bash
doclinks docs/guide.md   # single file
doclinks docs/           # directory
doclinks --dry-run ...   # preview only
```
