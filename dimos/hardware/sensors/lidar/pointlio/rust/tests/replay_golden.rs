// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// End to end: `run` the first frame of the 60 s file; preprocess + downsample must be exact.

use std::path::Path;
use std::process::Command;

use dimos_pointlio::golden::read_frames;

const DATA: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../../../../data");

#[test]
fn frame0_matches_golden() {
    let data = Path::new(DATA);
    let replay = data.join("pointlio_replay/mid360_athens_stairs_60s.plio");
    let golden = data.join("pointlio_golden/mid360_athens_stairs_60s");
    if !replay.exists() || !golden.join("frames.bin").exists() {
        eprintln!(
            "skip: {} or {} missing (gitignored data)",
            replay.display(),
            golden.display()
        );
        return;
    }
    let out = data.join("pointlio_golden/mid360_athens_stairs_60s_rust_test");
    let status = Command::new(env!("CARGO_BIN_EXE_pointlio_replay"))
        .arg("run")
        .args(["--replay".as_ref(), replay.as_os_str()])
        .args(["--config".as_ref(), golden.join("config.json").as_os_str()])
        .args(["--out".as_ref(), out.as_os_str()])
        .args(["--frames", "1", "--max-frames", "1"])
        .status()
        .unwrap();
    assert!(status.success());

    let g = &read_frames(golden.join("frames.bin")).unwrap()[0];
    let c = &read_frames(out.join("frames.bin")).unwrap()[0];
    assert_eq!(c.lidar_ts, g.lidar_ts);
    assert_eq!(c.feats_down_body.len(), g.feats_down_body.len(), "n_down");
    assert_eq!(c.feats_down_body, g.feats_down_body);
    assert_eq!(c.n_eff, g.n_eff);
}
