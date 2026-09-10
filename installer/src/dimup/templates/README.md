# APP_NAME

```bash
source .dimos/activate.sh
dimos run APP_NAME.demo
pytest
```

Edit `src/APP_MODULE/demo.py` to change the image producer or listener, then rerun
the application. Stop it with Ctrl-C. Add dependencies with `uv add <package>`.

This application is installed editable. Register additional blueprints under
`[project.entry-points."dimos.blueprints"]`, then run `uv sync` to refresh metadata.

Commit the source, manifest, lockfile, `.envrc`, and `.dimos` activation files.
On a new machine, run the DimOS bootstrap first. After cloning:

```bash
uv sync --locked
source .dimos/activate.sh
dimos run APP_NAME.demo
pytest
```

For automatic activation, install direnv, configure its shell hook, review
`.envrc`, and run `direnv allow`. Leaving the directory restores the previous
environment. Manual activation can be undone with `deactivate`.
