# Vendored: AprilTag reference C library

Upstream <https://github.com/AprilRobotics/apriltag>, commit `0e16a12dd380fd607e4afd54712ee9b1ffb9ec8f`
(2026-06-15). **Unmodified.** Licence BSD-2-Clause, `LICENSE.md`, kept alongside the sources — the
obligation is to reproduce the notice, which that file does.

## What was taken and what was left

Taken: `apriltag.c`, `apriltag_quad_thresh.c`, `apriltag.h`, `apriltag_math.h`,
`apriltag_pose.c`, `apriltag_pose.h`, the `tag36h11` family, and all of `common/`.

`apriltag_pose.[ch]` was **left behind at the first vendoring and added 2026-07-28**, from the
same pinned commit, by the re-vendoring procedure below (clone, checkout `0e16a12…`, verified
against the already-vendored files byte for byte before copying). The original reason for its
absence — "nothing here computes a pose, the intrinsics are not calibrated" — was half a
statement of fact and half a decision, and the decision changed: `tag_jni.c` now runs
`estimate_tag_pose_orthogonal_iteration` per detection, and the *un-calibrated intrinsics*
half is carried as gates and flags instead of as an absence (`metric` stays false, the solved
orientation is published only past the measured gates in `vision/TagPose`). It depends only on
`common/` pieces already vendored (`matd`, `homography`, `debug_print`).

Left behind still: every other tag family (`tag16h5`, `tag25h9`, `tagStandard41h12`, …), the
Python wrapper, the ArUco compatibility layer, the examples and the tests. Families are pure
data — 20 KB of codeword table each — and adding one back is a copy plus one line of CMake.

`common/getopt.[ch]` is dropped as well — it belongs to the command-line examples, not to the
detector. `common/postscript_utils.h` looks like it should go the same way and **must not**:
`apriltag.c` and `apriltag_quad_thresh.c` both include it unconditionally, for a debug output path
that is compiled whether or not `td->debug` is ever set.

## Why vendored rather than fetched

`FetchContent` would put a network fetch inside every `assembleDebug`, and the whole reason this
project pins its toolchain in `flake.nix` is that a build which reaches for the network is a build
that stops reproducing the day upstream retags. 684 KB of C in the tree is cheaper than that.

## Re-vendoring

```
git clone --depth 1 https://github.com/AprilRobotics/apriltag
cp apriltag.c apriltag.h apriltag_math.h apriltag_quad_thresh.c apriltag_pose.c apriltag_pose.h \
   tag36h11.c tag36h11.h LICENSE.md <here>/
cp -r common <here>/common
rm <here>/common/getopt.[ch] <here>/common/postscript_utils.h
```

Update the commit above when you do. `CMakeLists.txt` globs, so no file list needs touching.
