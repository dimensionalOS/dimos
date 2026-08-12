package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.vision.TagFix
import com.dimensional.mini4pro.vision.TagSighting

/**
 * **One tag sighting → one flight-record line**, as a pure function.
 *
 * The same shape and the same reason as [MavlinkTap]: `Recorder` needs Android and therefore cannot
 * be unit-tested, so anything in it that is a *decision* rather than plumbing has to live outside it
 * or nothing can hold it. This one has exactly two decisions in it and both are load-bearing.
 *
 * **It was extracted because a mutation survived.** With the mapping inline in `Recorder.tagSeen`,
 * changing `monoNanos = sighting.atNanos` to the recorder's own clock killed **zero** tests on
 * 2026-07-28 — the rule was stated in a comment and enforced by nothing. Moving eighteen lines here
 * made it one test.
 */
object TagTap {

    /**
     * @param sighting what the detector saw, in pixels and in the camera's frame.
     * @param fix where it is in `drone/world`, or null when it could not be worked out.
     * @param latched true on the single frame at which this became the flight's latched tag.
     */
    fun entry(
        sighting: TagSighting.Sighting,
        fix: TagFix?,
        latched: Boolean,
    ): LogEntry.Tag = LogEntry.Tag(
        // **Decision one: the frame's arrival time, not the write time.** By the time this line is
        // written the detection is 40–160 ms old — frame arrival every 41.5 ms, 25–50 ms to detect,
        // up to 125 ms waiting for the cap. Stamping it at the write would silently destroy every
        // latency measurement anyone could make from the record afterwards, including the one that
        // says how stale a sighting is when a controller uses it. It would also be invisible: the
        // numbers would still look plausible, just uniformly late.
        monoNanos = sighting.atNanos,
        tagId = sighting.tagId,
        centreX = sighting.centreX,
        centreY = sighting.centreY,
        pixelSize = sighting.pixelSize,
        width = sighting.imageWidth,
        height = sighting.imageHeight,
        hamming = sighting.hamming,
        decisionMargin = sighting.decisionMargin,
        x = sighting.x,
        y = sighting.y,
        z = sighting.z,
        metric = sighting.metric,
        northM = fix?.northM,
        eastM = fix?.eastM,
        fromHeightM = fix?.fromHeightM,
        // **Decision two: no fix means the bearing is assumed, not that it is known.** `true` is the
        // conservative reading of an absence, and the alternative — defaulting to `false` when there
        // is no fix to ask — would put "this bearing is measured" on the one line that has no
        // bearing at all.
        bearingAssumed = fix?.bearingAssumed ?: true,
        latched = latched,
        // No fix means no pitch was used at all, so there is nothing to attribute to the reported
        // angle: false, matching the field's own "older lines read as commanded" reading — a line
        // with no fix makes no pitch claim either way.
        pitchReported = fix?.pitchReported ?: false,
        // Corners and the raw solve, all-or-nothing per group, absent when the sighting has
        // none — which is both the legacy line (older readers see a valid line, older *lines*
        // read as "not measured") and the honest encoding of a detector without them. The solve
        // is written UNGATED: the record is where the gates stay measurable from flight data,
        // and belief is applied where a consumer can act on it (`TagPose.trusted`, at the
        // encoder), not at the recorder.
        c0x = sighting.corners?.x0,
        c0y = sighting.corners?.y0,
        c1x = sighting.corners?.x1,
        c1y = sighting.corners?.y1,
        c2x = sighting.corners?.x2,
        c2y = sighting.corners?.y2,
        c3x = sighting.corners?.x3,
        c3y = sighting.corners?.y3,
        qx = sighting.solve?.qx,
        qy = sighting.solve?.qy,
        qz = sighting.solve?.qz,
        qw = sighting.solve?.qw,
        tx = sighting.solve?.tx,
        ty = sighting.solve?.ty,
        tz = sighting.solve?.tz,
        e1 = sighting.solve?.err1,
        e2 = sighting.solve?.err2,
        tagM = sighting.solve?.tagSizeM,
        // The fix's own provenance, absence-conservative like bearingAssumed: no fix makes no
        // metric claim and carries no range. `== true` rather than `?: false` is the same
        // sentence; spelled this way so the null case reads as the decision it is.
        fixMetric = fix?.metric == true,
        rangeM = fix?.rangeM,
        // The ladder rung, verbatim and lowercase, absent exactly when the fix is — the bus's
        // `tag_fix` provenance token is derived from this field on both the live and the
        // offline path, which is what the record-first rule means by "derive both consumers".
        rangeSource = fix?.rangeSource?.name?.lowercase(),
    )
}
