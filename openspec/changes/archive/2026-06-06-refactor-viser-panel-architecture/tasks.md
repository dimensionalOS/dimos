## 1. Implementation

- [x] 1.1 `dimos/manipulation/viser_panel/animation.py`: Extract joint-path interpolation from `ViserManipulationPanelModule` - expect pure helper tests to pass unchanged.
- [x] 1.2 `dimos/manipulation/viser_panel/scene.py`: Extract Viser scene ownership for transform controls, URDF creation, target colors, joint mapping, and path line segments - expect scene tests to cover Viser handle mutations and `(N, 2, 3)` path segments.
- [x] 1.3 `dimos/manipulation/viser_panel/backend.py`: Extract manipulation backend facade for module-reference/RPC-client access, stale-client retry, IK/FK preview timeout mapping, planning, execution, cancel, and clear - expect backend tests with fake clients.
- [x] 1.4 `dimos/manipulation/viser_panel/gui.py`: Extract Viser GUI handle creation, dynamic joint sliders, callback wiring, and session-to-handle rendering - expect GUI tests to cover button enablement, robot/preset options, and callback intent dispatch.
- [x] 1.5 `dimos/manipulation/viser_panel/controller.py`: Extract workflow orchestration for refresh, robot selection, target changes, preset application, preview result application, plan, local preview, execute, cancel, and clear - expect controller tests to preserve target sync and gating behavior.
- [x] 1.6 `dimos/manipulation/viser_panel/module.py`: Reduce the module to config, DimOS lifecycle, dependency import, server creation, collaborator composition, polling thread, preview worker startup/shutdown, and RPC snapshot methods - expect public imports and blueprint symbols to remain stable.
- [x] 1.7 `dimos/manipulation/test_viser_panel.py`: Reorganize tests around `state`, `animation`, `scene`, `backend`, `gui`, `controller`, and public module regression coverage - expect existing behavior cases to remain represented.

## 2. Documentation

- [x] 2.1 `docs/capabilities/manipulation/readme.md`: Confirm the existing Viser panel docs still match the refactored package - expect no user-facing workflow changes.
- [x] 2.2 `docs/capabilities/manipulation/readme.md`: If key-file descriptions become stale, update them to describe the Viser panel package rather than a single-file implementation - expect doclinks to remain valid.

## 3. Verification

- [x] 3.1 Run `openspec validate refactor-viser-panel-architecture` - expect the change to be valid.
- [x] 3.2 Run `uv run pytest dimos/manipulation/test_viser_panel.py dimos/manipulation/test_manipulation_unit.py -q` - expect focused manipulation panel tests to pass.
- [x] 3.3 Run LSP diagnostics on every changed Python file under `dimos/manipulation/viser_panel/` and the updated tests - expect no new diagnostics.
- [x] 3.4 Run `uv run pre-commit run --all` - expect formatting, lint, docs, and repository hooks to pass.
- [x] 3.5 If docs changed, run `uv run pre-commit run doclinks --all-files` - expect doclinks to pass.
- [x] 3.6 Manually QA through the panel surface by launching the mock stack when no conflicting DimOS coordinator is running: `uv run --extra manipulation-viser dimos run xarm7-viser-panel-mock`, opening the Viser URL, selecting the robot, applying Current/Home/Init, moving Cartesian and joint targets, planning, local-previewing, and confirming Execute gating remains unchanged.
- [x] 3.7 If a conflicting DimOS coordinator is already running on the default bus, do not stop it without user approval; record that live panel QA was blocked by the existing coordinator and rely on focused tests plus a later isolated run.
