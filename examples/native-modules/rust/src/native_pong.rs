// NativeModule pong example.
//
// Receives Twist messages on `data` and echoes each one back on `confirm`,
// embedding the sample_config value in the reply's angular.z field.

use dimos_native_module::{LcmTransport, NativeModule};
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use serde::Deserialize;

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct PongConfig {
    sample_config: i64,
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    let (mut module, config) = NativeModule::from_stdin::<PongConfig>(transport)
        .await
        .expect("Failed to read config from stdin");

    eprintln!("pong: sample_config={}", config.sample_config);

    let mut data = module.input("data", Twist::decode);
    let confirm = module.output("confirm", Twist::encode);
    let _handle = module.spawn();

    eprintln!("pong ready");

    loop {
        match data.recv().await {
            Some(msg) => {
                let reply = Twist {
                    linear: msg.linear,
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: config.sample_config as f64,
                    },
                };
                confirm.publish(&reply).await.ok();
            }
            None => break,
        }
    }
}
