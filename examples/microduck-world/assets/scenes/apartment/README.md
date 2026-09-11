# Initial apartment scene

Copied from the DimOS Microduck four-room example at commit 536679f66d1f8c4a515edc9286c16ca3af4b89a0.
The original copyright and Apache-2.0 header are retained in scene.xml.

This project now owns this scene copy. scene.meta.json follows the existing DimOS
ScenePackage contract; places.json supplies room targets, aliases, landmarks,
scene identity and the robot spawn. Keep landmark positions consistent with the
named XML geoms. Robot model assets are fetched separately into assets/microduck
and retain their upstream license terms.

`viewer.json` owns browser-only appearance: background and ground colors, exposure,
per-geom color overrides keyed by XML geom name, and initial camera position/target.
Coordinates use MuJoCo meters and Z-up. Changes take effect after `./service restart world`;
the generated model URL changes automatically so browsers do not keep stale geometry.
The physics XML remains authoritative for collisions. To add a physical object, change
scene.xml; to change its browser color, use viewer.json. Textures and purely decorative
scene assets are future extensions of the project viewer, not demo modifications.
