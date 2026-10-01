# Recorder graph and isolated Python contracts

Point-LIO and FAST-LIO recorder ports now match their generated Odometry and
PointCloud2 producers. Their existing last-odometry pose anchoring is unchanged.
The RealSense/Mid360 assembly recorder and relocalization replay ports also
use generated message identities. Offline camera/relocalization viewer overrides
call external cloud helpers, and the standalone RealSense mount has an explicit
identity quaternion, with an offline identity-matrix check. No devices,
SLAM native processes or dataset replays were started.

Strict scoped mypy passed for these five production files. Three existing
RealSense configuration/registry tests passed; the actual generated mount matrix
and Rerun cloud helper were checked separately with a synthetic cloud. The four
new recorder/graph/render regression cases could not collect locally because
Recorder imports torch through EmbeddingModel; they remain pending CI, without
mocking torch or downloading models.

The isolated Python example's host contract and runtime now both use generated
Int32 and keyword scalar construction. Its existing process-level E2E producer
was migrated as well. **26 local tests passed** across module/bootstrap and a new
actual runtime callback test: input CDR is decoded, a negative value is multiplied,
and the generated output is independently re-decoded. No external worker was
started or environment installed locally. Process-level E2E remains a CI gate.

Exact prior published 23b8e1dd3fe3880c952acee39d860b7104c723b6 passed
codegen, strict lint, native, Rust, Web, and ARM. Python 3.10 had 6286 passed,
83 skipped and two dataset-export failures; 3.11/3.12 were cancelled.
The fixture now reaches export, where generic coercion had treated generated
Image as an object scalar. `resolve_field` explicitly views raw generated images
and decodes standard generated CompressedImage pixels. **36 dataprep tests
passed**, including CDR roundtrips and mono16 endian/row-padding behavior.
Strict scoped mypy passed for core.py. Full HDF5/LeRobot E2E still needs CI.

Historical lcm/jpeg SQLite codecs now give deliberate-cutover guidance, while
unknown codec IDs retain their distinct configuration error. No legacy decoding
or in-place rewriting was added. Model-dependent memory docs and the eight held
files remain paused, so this error improvement is not a passing md-babel claim.
