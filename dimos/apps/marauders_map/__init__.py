"""Harry-Potter "Marauder's Map" web app for the Go2.

A self-contained application built ON TOP of the jamjam target-lock /
distance-follow control stack (in ``dimos.robot.custom``). It composes those
modules into a blueprint and adds a parchment-styled web UI on port 7782 with:

* A live floor plan of detected people, each mapped to a stable HP character
* A click-to-track flow that drives the same BBoxSelectionModule the Rerun
  camera view already uses (no modifications to jamjam modules required --
  we synthesize a PointStamped on the existing ``clicked_point`` input)
* An on-page teleop pad (W/A/S/D/Q/E + STOP) that publishes Twist on
  ``tele_cmd_vel`` (MovementManager's existing priority lane)
* Hover tooltips, a full-roster gallery modal, HP-themed quote bubbles,
  per-person photo lightbox, and per-laptop SSID viewpoint labels

The whole app lives under this directory and depends only on public
interfaces of ``dimos.robot.custom`` -- so jamjam upstream can be
fast-forwarded with zero merge conflict here.
"""
