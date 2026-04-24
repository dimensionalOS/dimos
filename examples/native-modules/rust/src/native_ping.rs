// NativeModule ping example.
//
// Sends a Twist message at 5 Hz and logs each echo received on `confirm`.

use dimos_native_module::{LcmTransport, NativeModule};
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use tokio::time::{interval, Duration};

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    let (mut module, _config) = NativeModule::from_stdin::<()>(transport)
        .await
        .expect("Failed to read config from stdin");

    let mut confirm = module.input("confirm", Twist::decode);
    let data = module.output("data", Twist::encode);
    let _handle = module.spawn();

    let mut ticker = interval(Duration::from_millis(200));
    let mut seq = 0u64;

    loop {
        tokio::select! {
            _ = ticker.tick() => {
                let msg = Twist {
                    linear: Vector3 { x: seq as f64, y: 0.0, z: 0.0 },
                    angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                };
                data.publish(&msg).await.ok();
                seq += 1;
            }
            Some(echo) = confirm.recv() => {
                eprintln!("ping: echo received (seq={}, sample_config={})", echo.linear.x as u64, echo.angular.z as i64);
            }
        }
    }
}
