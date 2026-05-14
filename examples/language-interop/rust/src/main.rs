// Rust robot control example
// Subscribes to robot pose and publishes twist commands via LCM

use dimos_lcm::Lcm;
use lcm_msgs::geometry_msgs::{PoseStamped, Twist, Vector3};
use std::thread;
use std::time::{Duration, Instant};

const ODOM_CHANNEL: &str = "/odom#geometry_msgs.PoseStamped";
const CMD_VEL_CHANNEL: &str = "/cmd_vel#geometry_msgs.Twist";
const PUBLISH_INTERVAL: Duration = Duration::from_millis(100); // 10 Hz

fn main() {
    let lcm = Lcm::new().expect("Failed to create LCM transport");

    println!("Robot control started");
    println!("Subscribing to /odom, publishing to /cmd_vel");
    println!("Press Ctrl+C to stop.\n");

    let mut t: f64 = 0.0;
    let mut next_publish = Instant::now();

    loop {
        // Poll for incoming messages
        match lcm.try_recv() {
            Ok(Some(msg)) if msg.channel == ODOM_CHANNEL => {
                match PoseStamped::decode(&msg.data) {
                    Ok(pose) => {
                        let pos = &pose.pose.position;
                        let ori = &pose.pose.orientation;
                        println!(
                            "[pose] x={:.2} y={:.2} z={:.2} | qw={:.2}",
                            pos.x, pos.y, pos.z, ori.w
                        );
                    }
                    Err(e) => eprintln!("[pose] decode error: {e}"),
                }
            }
            Ok(Some(_)) => {} // ignore other channels
            Ok(None) => {}
            Err(e) => eprintln!("recv error: {e}"),
        }

        // Publish twist at 10 Hz
        let now = Instant::now();
        if now >= next_publish {
            let twist = Twist {
                linear: Vector3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: t.sin() * 0.3,
                },
            };

            let data = twist.encode();
            if let Err(e) = lcm.publish(CMD_VEL_CHANNEL, &data) {
                eprintln!("[twist] publish error: {e}");
            } else {
                println!(
                    "[twist] linear={:.2} angular={:.2}",
                    twist.linear.x, twist.angular.z
                );
            }

            t += 0.1;
            next_publish = now + PUBLISH_INTERVAL;
        }

        // Sleep until next publish deadline (capped at 10ms) to avoid busy-spinning
        let sleep_dur = next_publish.saturating_duration_since(Instant::now()).min(Duration::from_millis(10));
        thread::sleep(sleep_dur);
    }
}
