# hyperspace (vendored)

Library source vendored from **github.com/jeff-hykin/hyperspace** (private), commit `edae27c`.
Only the library is vendored: upstream's CLI binaries, three.js viewer and recording
exporters stay upstream. The dimos-facing wrapper is the parent crate,
[`dimos-hyperspace`](../).

Design and rationale live upstream in `plan.md` / `docs/principles.md`. The short version:

- Each SigLIP2 patch of a kept keyframe is a **hot pyramid** — a view frustum truncated at
  0.9x..1.1x the fused depth of that patch.
- A text query scores every stored patch in one matmul, keeps the hot ones, places their
  pyramids through the tf graph **at query time** (no pose is ever stored, so loop closure
  can rewrite the past), rasterizes them into a sparse voxel hash and pools per voxel:
  max over a frame's patches, log-sum-exp across frames, times sqrt of the number of
  distinct hot yaw bins.
- Keyframes are chosen by an 11-frame rolling buffer whose middle frame is judged against
  the 5 before and 5 after it, behind a per-camera quality gate.

To re-sync: copy `src/` from upstream and re-run this crate's tests. Keep the wrapper in the
parent crate so the vendored tree stays a clean copy.
