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

use std::time::{SystemTime, UNIX_EPOCH};

use dimos_module::nalgebra::Isometry3;
use dimos_module::{run_with_transport, Module, Tf, Transform};
use tokio::time::{interval, Duration};

#[derive(Module)]
#[module(setup = start_broadcast)]
struct TfBroadcaster {
    #[tf]
    tf: Tf,
}

impl TfBroadcaster {
    async fn start_broadcast(&mut self) {
        let tf = self.tf.clone();
        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(100));
            loop {
                ticker.tick().await;
                let ts = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .expect("system clock before epoch")
                    .as_secs_f64();
                let t = Transform::new("c", "d", ts, Isometry3::translation(0.5, 0.0, 0.0));
                if tf.publish(&[t]).await.is_err() {
                    break;
                }
            }
        });
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<TfBroadcaster>().await;
}
