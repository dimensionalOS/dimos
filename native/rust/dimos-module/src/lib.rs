pub mod lcm;
pub mod log;
pub mod module;
pub mod transport;
pub mod zenoh;

pub use dimos_module_macros::{native_config, Module};
pub use lcm::LcmTransport;
pub use module::{run, Builder, Input, Module, ModuleConfig, NativeConfig, NoConfig, Output};
pub use transport::Transport;
pub use zenoh::ZenohTransport;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;

/// Run module `M` over the transport named by the `DIMOS_TRANSPORT` env var.
///
/// Every transport is compiled in, so one binary follows whichever transport the
/// coordinator picks at runtime. The coordinator always sets the variable, so an
/// unset or unknown value is an error.
pub async fn run_with_transport<M: Module>() {
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => {
            run::<M, _>(
                LcmTransport::new()
                    .await
                    .expect("failed to create lcm transport"),
            )
            .await
        }
        Ok("zenoh") => {
            run::<M, _>(
                ZenohTransport::new()
                    .await
                    .expect("failed to create zenoh transport"),
            )
            .await
        }
        other => panic!("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"),
    }
}
