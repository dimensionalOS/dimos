pub mod lcm;
pub mod module;
pub mod transport;

pub use dimos_module_macros::Module;
pub use lcm::LcmTransport;
pub use module::{run, Builder, Input, Module, Output};
pub use transport::Transport;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;
