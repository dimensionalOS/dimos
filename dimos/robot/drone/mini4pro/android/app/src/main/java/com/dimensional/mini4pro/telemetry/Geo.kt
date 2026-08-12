package com.dimensional.mini4pro.telemetry

/**
 * What counts as a geodetic coordinate this bridge is willing to repeat.
 *
 * Plain Kotlin, no DJI and no Android, so the rule is unit-testable and so the two
 * places that need it can share one definition: `StateCache`, where DJI values are
 * unwrapped, and [TelemetryEncoder], the last gate before the wire.
 *
 * ## Why this exists — measured 2026-07-26 11:17
 *
 * `AircraftState` documents that **null means "no valid reading"**, and every
 * consumer trusts it: `HOME_POSITION`/`GPS_GLOBAL_ORIGIN` are suppressed on null
 * home, `GLOBAL_POSITION_INT` on null position. That contract assumed DJI signals
 * "I have no home point" with a null, the way it signals a missing component.
 *
 * It does not. With the aircraft powered, linked, and never flown this session,
 * `FlightControllerKey.KeyHomeLocation` delivered a **non-null**
 * `LocationCoordinate2D` whose latitude *and* longitude were both exactly
 * `4.583662361046586E7` — 45 836 623.61, which is not a latitude at all. DJI's own
 * `KeyIsHomeLocationSet` read `false` at the same instant, and in
 * `tmp/session-logs/20260725-191604.001.jsonl` the pair flipped together: at
 * t=1036.5 `isHomeLocationSet=false` with the 4.58e7 pair, at t=1091.2
 * `isHomeLocationSet=true` with 37.9938232, 23.7253477 (the real site). So the
 * garbage is DJI's placeholder for "not recorded yet", not a transient.
 *
 * Unchecked, that value reached the encoder as a known home, `degE7` saturated it
 * to `INT32_MAX`, and the bridge published 220 `HOME_POSITION` messages carrying
 * `latitude=2147483647`. Landing on the unknown sentinel was luck, not design.
 *
 * ## The rule, and why it is a range check
 *
 * 1. **Both coordinates present and finite.** Unchanged from before.
 * 2. **latitude in [-90, 90], longitude in [-180, 180].** This is the load-bearing
 *    check. It is deliberately *not* a comparison against `4.583662361046586E7`:
 *    a magic constant only holds until DJI changes the filler, while "a latitude
 *    outside ±90° is not a latitude" is true of every firmware that will ever
 *    exist. The bounds are inclusive — ±90 and ±180 are real, reachable places.
 * 3. **latitude != longitude, bit for bit.** A range check alone would pass a
 *    filler value that happened to land inside the bounds, and DJI's observed
 *    filler has a distinctive shape: *one number written into both fields*. A
 *    genuine GNSS solution produces two independent doubles, and the chance of
 *    those being bit-identical is on the order of 1e-16 per fix. This also
 *    excludes the exactly-0/0 pair, which the whole file already treats as the
 *    canonical "unknown encoded as zero" mistake. The cost is a measure-zero
 *    diagonal across the globe; the benefit is that a future in-range filler is
 *    still caught.
 *
 * A rejected pair is returned as null, which every existing consumer already
 * handles as "we do not know" — the honest outcome, and the one PLAN.md's honesty
 * boundaries require.
 */
object Geo {

    /** Inclusive bound on a WGS-84 latitude, in degrees. */
    const val MAX_LATITUDE_DEG = 90.0

    /** Inclusive bound on a WGS-84 longitude, in degrees. */
    const val MAX_LONGITUDE_DEG = 180.0

    /**
     * The pair as (latitude, longitude) degrees when it is a coordinate we are
     * willing to repeat, or null when any part of it is missing, non-finite, out
     * of range, or carries DJI's one-number-in-both-fields filler.
     *
     * All-or-nothing on purpose: a valid latitude beside a rejected longitude is
     * still a position a GCS would plot, in the wrong place.
     */
    fun coordinateOrNull(latitude: Double?, longitude: Double?): Pair<Double, Double>? {
        val lat = latitude ?: return null
        val lon = longitude ?: return null
        if (!lat.isFinite() || !lon.isFinite()) return null
        if (lat < -MAX_LATITUDE_DEG || lat > MAX_LATITUDE_DEG) return null
        if (lon < -MAX_LONGITUDE_DEG || lon > MAX_LONGITUDE_DEG) return null
        // DJI's no-home filler is the same number in both fields (4.583662361046586E7
        // on 2026-07-26). Real fixes are never bit-identical across the two axes.
        if (lat == lon) return null
        return lat to lon
    }

    /** True when [coordinateOrNull] would accept the pair. */
    fun isValid(latitude: Double?, longitude: Double?): Boolean =
        coordinateOrNull(latitude, longitude) != null

    // ── the flat-earth geodesy — one implementation, every caller ────────────────
    //
    // Unified here 2026-07-27. Before that the same equirectangular conversion existed
    // three times over (`RepositionGuidance.nedMetres`, `MissionGeo.distanceM`, this
    // file's `enuMetres`) plus an inverse (`RepositionGuidance.offsetCoordinate`), each
    // written by an agent that could not see the others. This package is the right home:
    // no DJI, no Android, no MAVLink, already where coordinate validity is decided, and
    // importable by `guided/`, `mission/` and `zenoh/` without any of them depending on
    // each other.

    /**
     * Metres per degree of latitude: `2πR/360` with R = 6 371 000 m (mean earth radius).
     * Against WGS-84 this is wrong by <0.3%, i.e. <30 cm over 100 m of travel — noise next
     * to `RepositionGuidance.R_ACCEPT_M`.
     *
     * `RepositionGuidance.METRES_PER_DEG` is an alias for this constant (kept because the
     * name is cited by tests and design docs). `MissionGeo.METRES_PER_DEG` is deliberately
     * **not**: it is a different number (111 320.0) and unifying it would move a plan-
     * admission gate by 0.11 %. See that constant's KDoc.
     */
    const val METRES_PER_DEG = 111_194.93

    /**
     * **The `cos(latitude)` term — the only copy of it in the bridge, on purpose.**
     *
     * The ratio of a degree of longitude to a degree of latitude at [atLatitudeDeg]: how much
     * shorter an east-west degree gets as you leave the equator. Every conversion in this file,
     * and `MissionGeo.distanceM`, multiplies (or, running backwards, divides) by exactly this.
     *
     * **This is the landmine.** Dropping or misplacing the factor is *exactly zero* error at
     * the equator and a **21 % east error at this project's home latitude** (38°N, cos 38° =
     * 0.788) — so it is invisible in any test written at the equator or with a degenerate
     * coordinate, and catastrophic in the real one. This project has been bitten by that class
     * of mistake three times. `GeoMetresTest` fails loudly and by name if it is dropped, and
     * `RepositionGuidanceTest`, `OrbitGuidanceTest`, `MissionStoreTest` and
     * `ZenohTelemetryEncoderTest` all pin it end to end at 38°N.
     *
     * Named for the axis it belongs to so that applying it to *north* reads as obviously wrong:
     * latitude degrees do not shorten.
     */
    fun longitudeScale(atLatitudeDeg: Double): Double =
        kotlin.math.cos(Math.toRadians(atLatitudeDeg))

    /**
     * A longitude difference normalised into ±180°, so a point across the antimeridian is a
     * short offset rather than a lap of the planet. The only copy of that normalisation.
     */
    fun wrappedLonDeltaDeg(fromLonDeg: Double, toLonDeg: Double): Double {
        var dLon = toLonDeg - fromLonDeg
        if (dLon > 180.0) dLon -= 360.0
        if (dLon < -180.0) dLon += 360.0
        return dLon
    }

    /**
     * The **NED** offset in metres from (`fromLat`, `fromLon`) to (`toLat`, `toLon`), as
     * **`(north, east)`** — the axis order `guided/` and `mission/` think in (north, east,
     * *down*), and the one `RepositionGuidance.nedMetres` has always returned.
     *
     * Equirectangular small-offset conversion, valid far beyond the Q1 100 m cap. The
     * `cos(latitude)` factor on east is [longitudeScale] and is evaluated at the **`from`**
     * point's latitude, which is what makes [offsetCoordinate] its inverse.
     *
     * @return `(north, east)` — **not** `(east, north)`; see [enuMetres] for that.
     */
    fun nedMetres(
        fromLatDeg: Double,
        fromLonDeg: Double,
        toLatDeg: Double,
        toLonDeg: Double,
    ): Pair<Double, Double> {
        val north = (toLatDeg - fromLatDeg) * METRES_PER_DEG
        val dLon = wrappedLonDeltaDeg(fromLonDeg, toLonDeg)
        val east = dLon * METRES_PER_DEG * longitudeScale(fromLatDeg)
        return north to east
    }

    /**
     * The **ENU** offset in metres from (`fromLat`, `fromLon`) to (`toLat`, `toLon`), as
     * **`(east, north)`** — the local-frame arithmetic for the Zenoh bus, which speaks
     * X-east / Y-north / Z-*up* (`docs/decisions/2026-07-27-zenoh-answers.md` §Z-6).
     *
     * The same numbers as [nedMetres] with the axes named the other way round, and defined in
     * terms of it so there is no second copy of the `cos(latitude)` factor to rot. Both
     * conventions are kept because both callers are right about their own frame, and quietly
     * handing one caller the other's axis order is precisely the bug this file exists to stop.
     *
     * @return `(east, north)` — **not** `(north, east)`; see [nedMetres] for that.
     */
    fun enuMetres(
        fromLatDeg: Double,
        fromLonDeg: Double,
        toLatDeg: Double,
        toLonDeg: Double,
    ): Pair<Double, Double> {
        val (north, east) = nedMetres(fromLatDeg, fromLonDeg, toLatDeg, toLonDeg)
        return east to north
    }

    /**
     * The inverse of [nedMetres]: the coordinate `(latitude, longitude)` that is [northM] /
     * [eastM] metres from (`lat`, `lon`), in the same NED convention.
     *
     * **The same [longitudeScale] factor and the same [METRES_PER_DEG], run backwards** — which
     * is the whole reason it lives beside the forward conversion rather than in `guided/`. If
     * the two halves ever stop sharing them the round trip stops closing, and for the orbit's
     * join target that means a point that is not on the circle (21 % short in east at 38°N).
     * `GeoMetresTest` closes the loop to well inside a centimetre at several latitudes.
     *
     * Uses the *base* point's latitude, exactly as [nedMetres] uses the *from* point's, so the
     * two are inverses of each other over the tens of metres this project works in.
     *
     * @return `(latitude, longitude)` degrees.
     */
    fun offsetCoordinate(
        latDeg: Double,
        lonDeg: Double,
        northM: Double,
        eastM: Double,
    ): Pair<Double, Double> {
        val lat = latDeg + northM / METRES_PER_DEG
        val lon = lonDeg + eastM / (METRES_PER_DEG * longitudeScale(latDeg))
        return lat to lon
    }
}
