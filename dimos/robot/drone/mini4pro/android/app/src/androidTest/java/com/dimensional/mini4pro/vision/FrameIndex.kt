package com.dimensional.mini4pro.vision

import java.io.File

/**
 * The recorder's own `frame` lines, read back so a recorded flight can be fed through a decoder
 * on the phone.
 *
 * The flight recorder writes video as an **index plus an elementary stream**, not a container
 * (`record/VideoSidecar`, `docs/flight-recording.md`): each frame gets one JSONL line carrying its
 * byte offset, its length, and the `t` it arrived at — on the *same* monotonic clock as the
 * telemetry. That is what makes this profiler possible without an aircraft: the bytes are exactly
 * the bytes MSDK delivered, in exactly the order it delivered them, and every frame already knows
 * what the aircraft was doing when it arrived.
 *
 * `tools/videoexport` reads the same two files the same way. This is its Kotlin half, deliberately
 * kept to the three fields a decoder needs rather than reproducing the whole schema — `replay/`
 * owns reading a flight record properly, and this is a measurement harness.
 *
 * Parsing is by hand rather than by `org.json` because the lines are machine-written, flat, and
 * numeric, and because a torn last line is *expected* (the recorder is append-only and a power cut
 * lands mid-line) — so an unparseable line is skipped rather than thrown on, exactly as
 * `tools/videoexport` does.
 */
internal object FrameIndex {

    /** One frame's place in the sidecar. [tSeconds] is the flight record's own clock. */
    data class Frame(
        val n: Long,
        val tSeconds: Double,
        val part: Int,
        val offset: Long,
        val length: Int,
        val keyFrame: Boolean,
    )

    /**
     * Every `frame` line in [jsonl], in time order.
     *
     * Only frames from a single [part] are returned. The sidecar rotates parts every 64 MB and a
     * decoder cannot be fed across a rotation boundary without reopening the file, which this
     * harness has no reason to do: the reference dataset is one part.
     */
    fun read(jsonl: File, part: Int = 1): List<Frame> {
        val out = ArrayList<Frame>(4096)
        jsonl.forEachLine { line ->
            // Cheap reject before any parsing: the overwhelming majority of lines in a flight
            // record are not frames, and this runs over a 3.6 MB file on a phone.
            if (line.contains("\"k\":\"frame\"")) {
                val f = parse(line)
                if (f != null && f.part == part) out.add(f)
            }
        }
        out.sortBy { it.tSeconds }
        return out
    }

    private fun parse(line: String): Frame? {
        val n = long(line, "n") ?: return null
        val t = double(line, "t") ?: return null
        val off = long(line, "off") ?: return null
        val len = long(line, "len") ?: return null
        val part = long(line, "part") ?: return null
        return Frame(
            n = n,
            tSeconds = t,
            part = part.toInt(),
            offset = off,
            length = len.toInt(),
            keyFrame = line.contains("\"key\":true"),
        )
    }

    private fun field(line: String, key: String): String? {
        val at = line.indexOf("\"$key\":")
        if (at < 0) return null
        var i = at + key.length + 3
        val start = i
        while (i < line.length && line[i] != ',' && line[i] != '}') i++
        return if (i > start) line.substring(start, i) else null
    }

    private fun long(line: String, key: String): Long? = field(line, key)?.toLongOrNull()

    private fun double(line: String, key: String): Double? = field(line, key)?.toDoubleOrNull()
}
