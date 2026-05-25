When writing or editing markdown documentation, use `doclinks` tool to resolve file references.

Full documentation if needed: [utils/docs/doclinks.md](https://github.com/dimensionalOS/dimos/blob/main/dimos/utils/docs/doclinks.md)

## Syntax


| Pattern     | Example                                             |
|-------------|-----------------------------------------------------|
| Code file   | `[`service/spec.py`]()` → resolves path             |
| With symbol | `Configurable` in `[`spec.py`]()` → adds `#L<line>` |
| Doc link    | `[Configuration](.md)` → resolves to doc            |

## Usage

```bash
doclinks docs/guide.md   # single file
doclinks docs/           # directory
doclinks --dry-run ...   # preview only
```
