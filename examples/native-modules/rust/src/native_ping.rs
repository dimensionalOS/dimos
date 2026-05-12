use dimos_native_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use tokio::time::{interval, Duration};

#[derive(Module)]
#[module(setup = start_publisher)]
struct Ping {
    #[input(decode = Twist::decode)]
    confirm: Input<Twist>,

    #[output(encode = Twist::encode)]
    data: Output<Twist>,
}

impl Ping {
    async fn start_publisher(&mut self) {
        let data = self.data.clone();
        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(200));
            let mut seq = 0u64;
            loop {
                ticker.tick().await;
                let msg = Twist {
                    linear: Vector3 {
                        x: seq as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                };
                data.publish(&msg).await.ok();
                seq += 1;
            }
        });
    }

    async fn handle_confirm(&mut self, echo: Twist) {
        println!(
            "ping: echo received (seq={}, sample_config={})",
            echo.linear.x as u64,
            echo.angular.z as i64,
        );
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    run::<Ping, _>(transport).await.expect("ping run failed");
}
