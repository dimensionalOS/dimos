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

//! Hyperspace as a dimos module: colour + depth + tf in, text queries in,
//! scored voxel clouds out.

use std::sync::{Arc, Mutex};

use dimos_module::{warn_throttled, Input, Module, Output};
use lcm_msgs::sensor_msgs::{CameraInfo, Image, PointCloud2};
use lcm_msgs::std_msgs::String as StringMsg;
use lcm_msgs::tf2_msgs::TFMessage;
use tracing::{info, warn};

use crate::config::{build_backends, Config};
use crate::convert;
use crate::query_request::QueryRequest;

/// Wall time, used only to stamp answers.
fn now_secs() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

#[derive(Module)]
#[module(name = "hyperspace", setup = init_state)]
pub struct Hyperspace {
    #[input(decode = Image::decode, handler = on_color)]
    color_image: Input<Image>,

    #[input(decode = Image::decode, handler = on_depth)]
    depth_image: Input<Image>,

    #[input(decode = CameraInfo::decode, handler = on_camera_info)]
    camera_info: Input<CameraInfo>,

    #[input(decode = CameraInfo::decode, handler = on_depth_camera_info)]
    depth_camera_info: Input<CameraInfo>,

    // A plain input rather than `#[tf]`: hyperspace keeps its own full-history tf
    // graph so a query resolves keyframe poses at query time, and a loop closure
    // that rewrites past transforms also moves past answers.
    #[input(decode = TFMessage::decode, handler = on_tf)]
    tf: Input<TFMessage>,

    // `{"id": 7, "text": "a chair"}`, or bare text when no id is needed.
    #[input(decode = StringMsg::decode, handler = on_query, msg = "String")]
    query: Input<StringMsg>,

    // Scored voxels. `header.seq` is the id of the query that asked, so answers
    // pair with requests without an RPC.
    #[output(encode = PointCloud2::encode)]
    query_result: Output<PointCloud2>,

    // Occupied voxels from the kept keyframes' depth, for context in a viewer.
    #[output(encode = PointCloud2::encode)]
    scene_map: Output<PointCloud2>,

    #[config]
    config: Config,

    // Built in setup. Shared because queries run on a blocking thread.
    state: Option<Arc<Mutex<hyperspace::Hyperspace>>>,

    // Keyframes kept since scene_map was last published.
    keyframes_since_scene: u32,
}

impl Hyperspace {
    async fn init_state(&mut self) {
        let (embedder, text_embedder, depth_fuser) = match build_backends(&self.config) {
            Ok(backends) => backends,
            Err(error) => {
                // A missing model directory should stop the module, not silently
                // degrade a running robot to meaningless answers.
                panic!("hyperspace backends: {error}");
            }
        };
        let state = hyperspace::Hyperspace::new(
            self.config.hyperspace_config(),
            embedder,
            text_embedder,
            depth_fuser,
        );
        info!(
            world_frame = %self.config.world_frame,
            voxel_size = self.config.voxel_size,
            models = !self.config.model_dir.is_empty(),
            "hyperspace ready",
        );
        self.state = Some(Arc::new(Mutex::new(state)));
    }

    fn state(&self) -> Arc<Mutex<hyperspace::Hyperspace>> {
        self.state.as_ref().expect("built in setup").clone()
    }

    async fn on_tf(&mut self, msg: TFMessage) {
        let state = self.state();
        let mut state = state.lock().expect("hyperspace lock");
        for transform in convert::transforms(&msg) {
            state.update(&transform);
        }
    }

    async fn on_camera_info(&mut self, msg: CameraInfo) {
        self.state()
            .lock()
            .expect("hyperspace lock")
            .set_camera_intrinsics(convert::intrinsics(&msg));
    }

    async fn on_depth_camera_info(&mut self, msg: CameraInfo) {
        self.state()
            .lock()
            .expect("hyperspace lock")
            .set_camera_intrinsics(convert::intrinsics(&msg));
    }

    async fn on_depth(&mut self, msg: Image) {
        match convert::depth_frame(&msg) {
            Ok(depth) => self
                .state()
                .lock()
                .expect("hyperspace lock")
                .add_depth(depth),
            Err(error) => warn_throttled!(
                std::time::Duration::from_secs(5),
                error = %error,
                "dropped a depth frame",
            ),
        }
    }

    async fn on_color(&mut self, msg: Image) {
        let frame = match convert::color_frame(&msg) {
            Ok(frame) => frame,
            Err(error) => {
                warn_throttled!(
                    std::time::Duration::from_secs(5),
                    error = %error,
                    "dropped a colour frame",
                );
                return;
            }
        };
        // Embedding runs inline: with the stub backend it is microseconds, and
        // with the real one the module's second thread keeps queries served.
        let kept = {
            let state = self.state();
            let mut state = state.lock().expect("hyperspace lock");
            match state.add_image(frame) {
                Ok(kept) => kept,
                Err(error) => {
                    warn_throttled!(
                        std::time::Duration::from_secs(5),
                        error = %error,
                        "dropped a colour frame",
                    );
                    return;
                }
            }
        };
        if !kept {
            return;
        }
        self.keyframes_since_scene += 1;
        if self.config.scene_emit_every == 0
            || self.keyframes_since_scene < self.config.scene_emit_every
        {
            return;
        }
        self.keyframes_since_scene = 0;
        let (voxels, voxel_size) = {
            let state = self.state();
            let state = state.lock().expect("hyperspace lock");
            (
                state.scene_voxels(&self.config.world_frame, self.config.scene_min_samples),
                state.config.voxel_size,
            )
        };
        if voxels.is_empty() {
            return;
        }
        let cloud = convert::scene_cloud(
            &voxels,
            voxel_size,
            &self.config.world_frame,
            convert::secs_to_time(convert::time_secs(&msg.header.stamp)),
        );
        self.scene_map.publish(&cloud).await.ok();
    }

    async fn on_query(&mut self, msg: StringMsg) {
        let request = match QueryRequest::parse(&msg.data) {
            Ok(request) => request,
            Err(error) => {
                warn!(error = %error, payload = %msg.data, "ignored a query");
                return;
            }
        };
        let frame = if request.frame.is_empty() {
            self.config.world_frame.clone()
        } else {
            request.frame.clone()
        };
        let state = self.state();
        let text = request.text.clone();
        let target = frame.clone();
        // Scoring every stored patch is seconds of CPU on a full map, so it must
        // not sit on a runtime worker.
        let answer = tokio::task::spawn_blocking(move || {
            state.lock().expect("hyperspace lock").query(&text, &target)
        })
        .await;
        let heatmap = match answer {
            Ok(Ok(heatmap)) => heatmap,
            Ok(Err(error)) => {
                warn!(error = %error, query = %request.text, "query failed");
                return;
            }
            Err(error) => {
                warn!(error = %error, "query task panicked");
                return;
            }
        };
        info!(
            id = request.id,
            query = %request.text,
            voxels = heatmap.voxels.len(),
            hot_patches = heatmap.stats.hot_patches,
            keyframes_placed = heatmap.stats.keyframes_placed,
            "answered",
        );
        let cloud = convert::heatmap_cloud(&heatmap, request.id, convert::secs_to_time(now_secs()));
        self.query_result.publish(&cloud).await.ok();
    }
}
