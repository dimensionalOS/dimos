## User-Facing Docs

Update the runnable-blueprint documentation or quick-reference table if it enumerates supported manipulators, including that OpenYAM is exactly one mock/planning/teleoperation integration using the DimOS-owned `yam_gripper.urdf.xacro` wrapper. It offers direct mock gripper control only, not animated or synchronized finger state.

## Contributor Docs

None. The existing LFS-description and blueprint-registry guidance applies unchanged.

## Coding-Agent Docs

None. The repository guidance already documents generated blueprint registries and LFS-backed assets.

## Doc Validation

Run the repository's applicable documentation link validation for any changed Markdown documentation. Run `pytest dimos/robot/test_all_blueprints_generation.py` to validate the generated runnable-blueprint listing.

## No Docs Needed

No new conceptual, API, hardware-driver, or coding-agent documentation is needed. The change reuses existing manipulation workflows and adds no physical hardware support; only the user-facing availability listing may need an update.
