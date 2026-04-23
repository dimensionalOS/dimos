This mostly to track when conventions change (with regard to codebase updates) because this codebase is under heavy development. Note: this is a non-exhaustive list of conventions.

- When global_config.py shouldn't accidentally/indirectly import heavy libraries like rerun. But sometimes global_config needs the type definition or default value from a module. Preferably we import from the module file directly, however when thats not possible, we create a config.py for just that module's config and import that into global_config.py.
- When adding visualization tools to a blueprint/autoconnect, instead of using RerunBridge or WebsocketVisModule directly we should always use `vis_module`, which right now should look something like `vis_module(viewer_backend=global_config.viewer, rerun_config={}),`
- `DEFAULT_THREAD_JOIN_TIMEOUT` is used for all thread.join timeouts
- Don't use print inside of tests
- Module configs should be specified as `config: ModuleSpecificConfigClass`
- To customize the way rerun renders something, right now we use a `rerun_config` dict. This will (hopefully) change very soon to be a per-module config instead of a per-blueprint config
- Similar to the `rerun_config` the `rrb` (rerun blueprint) is defined at a blueprint level right now, but ideally would be a per-module contribution with only a per-blueprint override of the layout.
- No `__init__.py` files
- Helper blueprints (like `_with_vis`) that should not be used on their own need to start with an underscore to avoid being picked up by the all_blueprints.py code generation step
