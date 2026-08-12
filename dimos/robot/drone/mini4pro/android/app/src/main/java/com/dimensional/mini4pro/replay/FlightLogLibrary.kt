package com.dimensional.mini4pro.replay

import android.content.Context
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Recorder
import java.io.File

/**
 * The phone's own flight logs, as a list an operator can pick from — and the loader that turns
 * one into a [ReplayPlayer].
 *
 * The only Android-facing part of the replay path, and it is deliberately thin: it finds files
 * and reads them. Everything about *what a recording means* is [FlightRecordReader] and
 * [FlightReplay], which are pure and already tested against a real 371-second orbit; nothing
 * here re-parses anything or invents a second reader.
 *
 * ## Reading is not free, and must not happen on the main thread
 *
 * The reference session is 8.3 MB of which 21 303 lines are MAVLink hex this replay does not
 * want. [FlightRecordReader] skips those on a substring test *before* parsing, which is most of
 * the cost avoided — but "most of the cost of eight megabytes" is still not a main-thread
 * operation. [load] is written to be called from a background dispatcher and says so in its
 * signature by returning a plain value rather than posting anything.
 */
object FlightLogLibrary {

    /** What the recorder writes. */
    const val EXTENSION = ".jsonl"

    /** One entry in the picker. */
    data class Entry(
        val file: File,
        /** The recorder's session name plus its part number, e.g. `20260727-104512.001`. */
        val name: String,
        val bytes: Long,
        val modifiedAtMs: Long,
    ) {
        /** `20260727-104512.001 · 8.3 MB` — what the operator actually chooses between. */
        fun label(): String {
            val mb = bytes / (1024.0 * 1024.0)
            val size = if (mb >= 0.1) "%.1f MB".format(mb) else "%d kB".format(bytes / 1024)
            return "$name  ·  $size"
        }
    }

    /**
     * Every recording on the phone, **newest first** — the order an operator wants, because the
     * flight they are asking about is almost always the one that just happened.
     *
     * Empty when the directory does not exist, which is the honest answer for a phone that has
     * never recorded rather than an error to raise.
     */
    fun list(context: Context): List<Entry> {
        val dir = Recorder.directory(context)
        val files = dir.listFiles { f -> f.isFile && f.name.endsWith(EXTENSION) } ?: return emptyList()
        return files
            .map { Entry(it, it.name.removeSuffix(EXTENSION), it.length(), it.lastModified()) }
            .sortedByDescending { it.modifiedAtMs }
    }

    /**
     * Loads one recording into a cursor over its reconstructed states.
     *
     * **Call this off the main thread** — see the class doc.
     *
     * Damaged trailing lines are skipped rather than refusing the file: a session cut short by a
     * dead battery is exactly the flight somebody wants to look at, and throwing it away to
     * protect its ending would be the wrong trade. A file with no `dji_state` lines at all
     * yields an empty player, which the UI reports as "nothing to replay" rather than as a
     * blank picture of somewhere.
     */
    fun load(entry: Entry): ReplayPlayer = load(entry.file, entry.name)

    /** As [load], for a file chosen some other way. */
    fun load(file: File, name: String = file.name): ReplayPlayer {
        val record = file.bufferedReader().use { reader ->
            FlightRecordReader.read(
                reader.lineSequence(),
                // **`REPLAY_KINDS` plus `tag`**, since 2026-07-28. The detector's sightings are a
                // second stream a published replay can carry, and they are read here rather than
                // in a second pass because eight megabytes is a cost worth paying once. A record
                // with no `tag` lines simply yields no sightings; nothing else about the load
                // changes, and neither does the cost of a session that has none.
                FlightRecordReader.Options(
                    kinds = FlightRecordReader.REPLAY_KINDS + LogEntry.KIND_TAG,
                ),
            )
        }
        return ReplayPlayer(FlightReplay.samples(record), name, FlightReplay.sightings(record))
    }
}
