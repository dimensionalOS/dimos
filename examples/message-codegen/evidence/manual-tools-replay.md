# Manual tools and SQLite replay cutover

The pending tool batch now uses generated messages and external image, cloud,
pose and viewer adapters. The VLM batch tool requires `DIMOS_VLM_RECORDING`
pointing to `<path>.db/<image_stream>` containing generated CDR images; it no
longer assumes a historical typed-pickle recording is compatible.

Offline verification: 40 tests passed across `test_sqlite_moment.py`,
`test_generated_views.py` and `test_generated_cloud_bridge.py`. Temporary SQLite
recordings verify absolute database paths, CDR values, seek/window looping,
missing frames and SensorMoment publication. File image tests preserve RGB
values and the supplied nanosecond header. Explicit cloud RGB/RGBA colors stay
aligned after nonfinite/height filtering and reject invalid channels.

Scoped strict mypy passed seven files: image.py, moment.py, replay.py,
message_helpers.py, tool_visualizer.py, tool_localize.py and tool_vlm.py.
Required hooks passed after formatting. Manual broker/model/robot tools were
not executed; this evidence establishes their shared offline helpers, not
end-to-end broker or hardware behavior. Historical Go2 recordings still need
replacement before the self-hosted replay tools can run on new-format data.

At published HEAD `11bae55835e56c33f3940a9563cf15cfc81993b2`, codegen CI
36760272088 completed successfully (standalone and independent Jazzy reference).
Main CI 36760267791 remained in progress at this checkpoint: Rust, native,
Web and docs passed; lint was cancelled and Python lanes were still running.
This is not an all-green or final migration acceptance claim.
