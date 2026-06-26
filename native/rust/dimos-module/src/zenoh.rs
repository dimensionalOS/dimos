use std::io;

use ::zenoh::Session;

use crate::transport::{Dispatch, Transport};

/// Zenoh transport. Each subscribed channel is its own per-key Zenoh
/// subscription, so the network only delivers topics the module asked for.
pub struct ZenohTransport {
    session: Session,
}

impl ZenohTransport {
    pub async fn new() -> io::Result<Self> {
        let session = ::zenoh::open(::zenoh::Config::default())
            .await
            .map_err(to_io)?;
        Ok(Self { session })
    }
}

impl Transport for ZenohTransport {
    async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()> {
        self.session
            .put(key_for(channel), data)
            .await
            .map_err(to_io)
    }

    async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()> {
        self.session
            .declare_subscriber(key_for(channel))
            .callback(move |sample| on_msg(&sample.payload().to_bytes()))
            .background()
            .await
            .map_err(to_io)
    }
}

/// Convert a dimos topic to a Zenoh key: drop the leading `/`, and turn the
/// `#type` separator into a path segment (`#` is forbidden in Zenoh keys).
fn key_for(channel: &str) -> String {
    channel
        .strip_prefix('/')
        .unwrap_or(channel)
        .replace('#', "/")
}

fn to_io(e: ::zenoh::Error) -> io::Error {
    io::Error::other(e)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::time::Duration;

    #[test]
    fn strips_leading_slash() {
        assert_eq!(key_for("/foo/data"), "foo/data");
    }

    #[test]
    fn leaves_unprefixed_unchanged() {
        assert_eq!(key_for("foo/data"), "foo/data");
    }

    #[test]
    fn nested_topic() {
        assert_eq!(key_for("/robot/cmd_vel"), "robot/cmd_vel");
    }

    #[test]
    fn type_suffix_becomes_path_segment() {
        assert_eq!(
            key_for("/data#geometry_msgs.Twist"),
            "data/geometry_msgs.Twist"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn round_trip_delivers_payload() {
        let transport = ZenohTransport::new().await.expect("open session");

        let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(8);
        let sink: Dispatch = Arc::new(move |bytes: &[u8]| {
            let _ = tx.try_send(bytes.to_vec());
        });
        transport
            .subscribe("/dimos_test/round_trip", sink)
            .await
            .expect("subscribe");

        let payload = b"hello zenoh";
        // Publish until the subscriber sees it, to tolerate subscription setup latency.
        let received = tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                transport
                    .publish("/dimos_test/round_trip", payload.to_vec())
                    .await
                    .expect("publish");
                if let Ok(Some(got)) =
                    tokio::time::timeout(Duration::from_millis(100), rx.recv()).await
                {
                    break got;
                }
            }
        })
        .await
        .expect("payload not delivered within timeout");

        assert_eq!(received, payload);
    }
}
