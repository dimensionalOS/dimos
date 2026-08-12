package com.dimensional.mini4pro.video

/**
 * The ordering rule, made explicit and testable.
 *
 * Touching the MSDK before registration completes **silently does nothing** — no
 * error, no callback (see docs/architecture.md and docs/dev-environment.md). The
 * live-stream manager adds a second precondition: no aircraft means no camera
 * stream to serve, and `startStream()` on a disconnected product either fails
 * with an opaque error or reports success and serves nothing.
 *
 * So the preconditions are checked in a fixed order and the *reason* we are not
 * yet running is a first-class value, not a boolean. "Nothing happened" must
 * always come with which gate we are stuck behind.
 */
enum class VideoGate {
    /** Every precondition met; the stream may be brought up. */
    READY,

    /** `Msdk.state.registered == false`. Nothing may touch the SDK yet. */
    WAIT_REGISTRATION,

    /** Registered, but no aircraft on the cable. */
    WAIT_AIRCRAFT,

    /** Aircraft connected, but MSDK has not reported the camera yet. */
    WAIT_CAMERA,
    ;

    companion object {
        /**
         * @param registered `Msdk.state.value.registered`
         * @param aircraftConnected `Msdk.state.value.productConnected`
         * @param cameraAvailable the camera index appears in
         *   `ICameraStreamManager.AvailableCameraUpdatedListener`'s list, **or**
         *   `CameraKey.KeyConnection` reads true. Either is enough; both are
         *   reported separately in [VideoStatus] so a stall is diagnosable.
         */
        fun evaluate(
            registered: Boolean,
            aircraftConnected: Boolean,
            cameraAvailable: Boolean,
        ): VideoGate = when {
            !registered -> WAIT_REGISTRATION
            !aircraftConnected -> WAIT_AIRCRAFT
            !cameraAvailable -> WAIT_CAMERA
            else -> READY
        }
    }

    /** The phase to report while waiting behind this gate. */
    fun toPhase(): VideoPhase = when (this) {
        READY -> VideoPhase.STARTING
        WAIT_REGISTRATION -> VideoPhase.WAITING_REGISTRATION
        WAIT_AIRCRAFT -> VideoPhase.WAITING_AIRCRAFT
        WAIT_CAMERA -> VideoPhase.WAITING_CAMERA
    }
}
