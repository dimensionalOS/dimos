use dimos_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use serde::Deserialize;

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct PongConfig {
    sample_config: i64,
}

#[derive(Module)]
struct Pong {
    #[input(decode = Twist::decode)]
    data: Input<Twist>,

    #[output(encode = Twist::encode)]
    confirm: Output<Twist>,

    #[config]
    config: PongConfig,
}

impl Pong {
    async fn handle_data(&mut self, msg: Twist) {
        let reply = Twist {
            linear: msg.linear,
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: self.config.sample_config as f64,
            },
        };
        self.confirm.publish(&reply).await.ok();
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    run::<Pong, _>(transport).await.expect("pong run failed");
}
