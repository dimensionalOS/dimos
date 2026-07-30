(doc-development-conventions-conventions)=

# Conventions

This mostly to track when conventions change (with regard to codebase updates) because this codebase is under heavy development. Note: this is a non-exhaustive list of conventions.

- Instead of using `RerunBridge` in blueprints we always use [`vis_module`][vis_module] which allows the CLI to control if its rerun or no-vis at all
- `global_config.py` should not accidentally or indirectly import heavy libraries like Rerun. Sometimes global config needs a type definition or default value from a module. Prefer importing from the module file directly. When that is not possible, create a `config.py` for the module's configuration and import that into `global_config.py`.
- When adding visualization tools to a blueprint/autoconnect, instead of using RerunBridge or WebsocketVisModule directly we should always use [`vis_module`][vis_module], which right now should look something like `vis_module(viewer_backend=global_config.viewer, rerun_config={}),`
- `DEFAULT_THREAD_JOIN_TIMEOUT` is used for all thread.join timeouts
- Don't use print inside of tests
- Module configs should be specified as `config: ModuleSpecificConfigClass`
- To customize the way rerun renders something, right now we use a `rerun_config` dict. This will (hopefully) change very soon to be a per-module config instead of a per-blueprint config
- Similar to the `rerun_config` the `rrb` (rerun blueprint) is defined at a blueprint level right now, but ideally would be a per-module contribution with only a per-blueprint override of the layout.
- No `__init__.py` files
- Helper blueprints (like `_with_vis`) that should not be used on their own need to start with an underscore to avoid being picked up by the in-repo `all_blueprints.py` code generation step
- Built-in runnable blueprints are registered through the generated in-repo `all_blueprints.py`; externally packaged blueprints are discovered through installed Python package entry points in the `dimos.blueprints` group instead.

[vis_module]: #dimos.visualization.vis_module.vis_module
