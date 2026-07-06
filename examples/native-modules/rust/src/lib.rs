use dimos_module::{run, LcmTransport, Module, ZenohTransport};

/// Run module `M` over the transport named by `DIMOS_TRANSPORT`.
///
/// Every transport is compiled into the binary, so the same build follows
/// whichever transport the coordinator picks at runtime. The coordinator
/// always sets the variable, so an unset or unknown value is an error.
pub async fn run_over_selected_transport<M: Module>() {
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => {
            run::<M, _>(
                LcmTransport::new()
                    .await
                    .expect("failed to create lcm transport"),
            )
            .await;
        }
        Ok("zenoh") => {
            run::<M, _>(
                ZenohTransport::new()
                    .await
                    .expect("failed to create zenoh transport"),
            )
            .await;
        }
        other => panic!("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"),
    }
}
