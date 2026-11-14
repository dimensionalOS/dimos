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

use dimos_module::{run_with_transport, Module, Tf, Transform};
use tokio::time::{interval, Duration};

#[derive(Module)]
#[module(setup = start_lookup)]
struct TfListener {
    // `handler = ...` also hands each message to that method; a plain `#[tf]`
    // only fills the graph.
    #[tf(handler = handle_tf)]
    tf: Tf,
}

impl TfListener {
    async fn handle_tf(&mut self, transforms: Vec<Transform>) {
        for t in &transforms {
            tracing::debug!(parent = %t.parent, child = %t.child, "tf edge received");
        }
    }

    async fn start_lookup(&mut self) {
        let tf = self.tf.clone();
        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(500));
            loop {
                ticker.tick().await;
                match tf.get_latest("a", "d") {
                    Some(t) => {
                        let p = t.translation();
                        tracing::info!(
                            parent = %t.parent,
                            child = %t.child,
                            x = p.x,
                            y = p.y,
                            z = p.z,
                            "tf lookup",
                        );
                    }
                    None => tracing::info!("a -> d not available yet"),
                }
            }
        });
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<TfListener>().await;
}
