pub mod lcm;
pub mod module;
pub mod transport;

pub use lcm::LcmTransport;
pub use module::{Input, NativeModule, NativeModuleHandle, Output};
pub use transport::Transport;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;
