package com.dimensional.mini4pro.video

/**
 * The single-lens trap, as a decision function.
 *
 * The Mini 4 Pro has **one** lens. `CameraVideoStreamSourceType.WIDE_CAMERA`
 * exists in the enum and MSDK accepts it, but selecting it kills the stream with
 * no error at all — this was the root cause in MSDK issue #641, where a
 * developer got zero frames and no diagnostic.
 *
 * Rather than hardcode `DEFAULT_CAMERA` and hope, we read
 * `CameraKey.KeyCameraVideoStreamSourceRange` and let the airframe tell us. Our
 * own sweep on the real aircraft
 * (docs/measurements/2026-07-25-sweep-readable-raw.txt) reports:
 *
 * ```
 * Camera.KeyCameraVideoStreamSource      = DEFAULT_CAMERA
 * Camera.KeyCameraVideoStreamSourceRange = [DEFAULT_CAMERA]
 * ```
 *
 * i.e. exactly one legal value. So on a healthy Mini 4 Pro the plan is always
 * [Action.KEEP] and we write nothing — a needless `setValue` on a live stream is
 * itself a way to break it.
 *
 * Kept DJI-free (source types are enum *names*) so the logic is unit-testable.
 */
object StreamSource {

    /** The only source a single-lens airframe can serve. */
    const val SINGLE_LENS = "DEFAULT_CAMERA"

    /** The source that silently kills the stream on this airframe. See #641. */
    const val KILLS_THE_STREAM = "WIDE_CAMERA"

    enum class Action {
        /** Current source is fine. Write nothing. */
        KEEP,

        /** Current source is wrong or unset; write [Plan.desired]. */
        SWITCH,
    }

    data class Plan(
        val action: Action,
        /** Enum name to write, or null when [action] is [Action.KEEP]. */
        val desired: String?,
        /** Non-null when the operator needs to know something. Surfaced in status. */
        val warning: String?,
    )

    /**
     * @param range enum names from `KeyCameraVideoStreamSourceRange`; empty when
     *   the key could not be read (which happens before the camera reports in).
     * @param current enum name from `KeyCameraVideoStreamSource`, or null.
     */
    fun plan(range: List<String>, current: String?): Plan {
        val clean = range.filter { it.isNotBlank() && it != "UNKNOWN" }

        // Range unreadable. Do not guess a lens — but do rescue the one case we
        // know is fatal: a stale WIDE_CAMERA selection means no frames, forever.
        if (clean.isEmpty()) {
            return if (current == SINGLE_LENS) {
                Plan(Action.KEEP, null, "stream-source range unreadable; source already $SINGLE_LENS")
            } else {
                Plan(
                    Action.SWITCH,
                    SINGLE_LENS,
                    "stream-source range unreadable and source is ${current ?: "unset"}; " +
                        "forcing $SINGLE_LENS (single-lens airframe, MSDK #641)",
                )
            }
        }

        val warnings = mutableListOf<String>()
        if (clean.size > 1) {
            // Measured range on the Mini 4 Pro is exactly [DEFAULT_CAMERA]. More
            // than one means this is not the airframe we measured.
            warnings += "airframe reports ${clean.size} stream sources $clean; " +
                "the Mini 4 Pro measured exactly [$SINGLE_LENS]"
        }

        val target = if (SINGLE_LENS in clean) {
            SINGLE_LENS
        } else {
            warnings += "$SINGLE_LENS is not in the supported range $clean"
            clean.first()
        }

        return if (current == target) {
            Plan(Action.KEEP, null, warnings.joinToString("; ").ifEmpty { null })
        } else {
            if (current == KILLS_THE_STREAM) {
                warnings += "current source is $KILLS_THE_STREAM, which yields zero frames " +
                    "on a single-lens airframe (MSDK #641)"
            }
            Plan(Action.SWITCH, target, warnings.joinToString("; ").ifEmpty { null })
        }
    }
}
