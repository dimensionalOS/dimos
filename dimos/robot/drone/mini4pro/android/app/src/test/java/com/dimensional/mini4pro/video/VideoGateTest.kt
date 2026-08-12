package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * The ordering rule that has already cost a debugging cycle in this codebase:
 * nothing may touch the MSDK before registration completes, and aircraft state
 * needs the product connected too.
 */
class VideoGateTest {

    @Test
    fun `registration is checked first, whatever the other flags claim`() {
        // A stale productConnected must not let us through: touching the SDK
        // unregistered silently does nothing, which is the worst outcome.
        assertEquals(
            VideoGate.WAIT_REGISTRATION,
            VideoGate.evaluate(registered = false, aircraftConnected = true, cameraAvailable = true),
        )
    }

    @Test
    fun `aircraft is checked before the camera`() {
        assertEquals(
            VideoGate.WAIT_AIRCRAFT,
            VideoGate.evaluate(registered = true, aircraftConnected = false, cameraAvailable = true),
        )
    }

    @Test
    fun `camera is the last gate`() {
        assertEquals(
            VideoGate.WAIT_CAMERA,
            VideoGate.evaluate(registered = true, aircraftConnected = true, cameraAvailable = false),
        )
    }

    @Test
    fun `all three met is ready`() {
        assertEquals(
            VideoGate.READY,
            VideoGate.evaluate(registered = true, aircraftConnected = true, cameraAvailable = true),
        )
    }

    @Test
    fun `every gate maps to a distinct reported phase`() {
        assertEquals(VideoPhase.WAITING_REGISTRATION, VideoGate.WAIT_REGISTRATION.toPhase())
        assertEquals(VideoPhase.WAITING_AIRCRAFT, VideoGate.WAIT_AIRCRAFT.toPhase())
        assertEquals(VideoPhase.WAITING_CAMERA, VideoGate.WAIT_CAMERA.toPhase())
        assertEquals(VideoPhase.STARTING, VideoGate.READY.toPhase())
        assertEquals(
            VideoGate.values().size,
            VideoGate.values().map { it.toPhase() }.toSet().size,
        )
    }
}
