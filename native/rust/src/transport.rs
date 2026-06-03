use std::future::Future;
use std::io;

/// Abstraction over the message transport used by a native module.
///
/// New transport protocols should implement this trait.
/// `NativeModule` is generic over any transport
pub trait Transport: Send + 'static {
    /// Send `data` on `channel`.
    fn publish(&self, channel: &str, data: &[u8]) -> impl Future<Output = io::Result<()>> + Send;
    /// Block until the next inbound message, returning `(channel, data)`.
    fn recv(&mut self) -> impl Future<Output = io::Result<(String, Vec<u8>)>> + Send;
}
