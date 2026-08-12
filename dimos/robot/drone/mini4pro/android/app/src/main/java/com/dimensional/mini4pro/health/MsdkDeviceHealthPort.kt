package com.dimensional.mini4pro.health

import com.dimensional.mini4pro.Msdk
import com.dimensional.mini4pro.warn.WarnLevel
import com.dimensional.mini4pro.warn.WarnSource
import com.dimensional.mini4pro.warn.Warning
import dji.v5.manager.diagnostic.DJIDeviceHealthInfo
import dji.v5.manager.diagnostic.DJIDeviceHealthInfoChangeListener
import dji.v5.manager.diagnostic.DeviceHealthManager

/**
 * The DJI half of device health: `DeviceHealthManager` in, [Warning] out. The only file in the
 * package that imports a DJI type.
 *
 * ## The API surface, verified
 *
 * `javap` against `dji-sdk-v5-aircraft-provided-5.18.0.jar`, 2026-07-26 — not taken from
 * documentation, which does not cover this manager's method set completely:
 *
 * ```
 * // dji.v5.manager.diagnostic.DeviceHealthManager  (implements IDeviceHealthManager)
 * public static IDeviceHealthManager getInstance();
 * public void addDJIDeviceHealthInfoChangeListener(DJIDeviceHealthInfoChangeListener);
 * public void removeDJIDeviceHealthInfoChangeListener(DJIDeviceHealthInfoChangeListener);
 * public List<DJIDeviceHealthInfo> getCurrentDJIDeviceHealthInfos();
 * public void init();  public void destroy();  public void clearAllListeners();
 *
 * // dji.v5.manager.diagnostic.DJIDeviceHealthInfoChangeListener  — a SAM interface
 * void onDeviceHealthInfoUpdate(List<DJIDeviceHealthInfo>);
 *
 * // dji.v5.manager.diagnostic.DJIDeviceHealthInfo  (implements IDJIDeviceHealthInfo)
 * String informationCode();  int componentId();  int sensorIndex();
 * String title();  String description();  WarningLevel warningLevel();
 *
 * // dji.v5.manager.diagnostic.WarningLevel
 * NORMAL(0) NOTICE(1) CAUTION(2) WARNING(3) SERIOUS_WARNING(4) UNKNOWN(65535)
 * ```
 *
 * Three things that surface has that we deliberately do not call:
 *
 *  - **`init()` / `destroy()`.** DJI's own sample never calls either
 *    (`SampleCode-V5/.../DiagnosticVm.kt`, and the UX SDK's
 *    `DeviceHealthAndStatusWidgetModel.kt` likewise), and `getInstance()` is a lazy-holder
 *    singleton that the SDK brings up itself. Calling `destroy()` on a process-wide singleton
 *    from a bridge that is one of several SDK consumers would be reaching outside our own
 *    lifetime.
 *  - **`clearAllListeners()`**, for the same reason with a sharper edge: it would remove
 *    listeners we did not add.
 *
 * ## Nulls
 *
 * Every accessor is a Java method with no nullability annotation, so Kotlin sees them as
 * platform types and would happily assign a `null` into a non-null `String`. `title()` and
 * `description()` are built by table lookup (`DJIDeviceHealthInfo.updateDesDescription`) and are
 * genuinely absent for a code DJI has no entry for, so both are read defensively. The whole
 * conversion is wrapped: a malformed entry must not take out a DJI callback on the main thread,
 * and a single bad item must not cost us the rest of the list.
 */
class MsdkDeviceHealthPort(
    private val log: (String) -> Unit = {},
) : DeviceHealthPort {

    private var listener: DJIDeviceHealthInfoChangeListener? = null

    /**
     * Registration only — deliberately **not** product connection.
     *
     * The gimbal and virtual-stick ports also refuse without a product, because both are about to
     * command one. This one only listens, and a health picture is worth having the instant the
     * SDK can deliver it: waiting for `productConnected` would mean re-planting on every
     * reconnect for no gain, and DJI's list is empty when there is no aircraft anyway.
     */
    override fun unavailableReason(): String? =
        if (Msdk.state.value.registered) null else "SDK_NOT_REGISTERED"

    override fun listen(onDelivery: (List<Warning>) -> Unit): Boolean {
        val l = DJIDeviceHealthInfoChangeListener { infos -> onDelivery(convert(infos)) }
        return try {
            DeviceHealthManager.getInstance().addDJIDeviceHealthInfoChangeListener(l)
            // Only after the SDK has taken it: `listener` is what `cancelListen` removes, and
            // recording one the SDK never accepted would make teardown lie about what it undid.
            listener = l
            true
        } catch (t: Throwable) {
            // Contained for the reason the interface states: this runs on the bridge's tick, and
            // an escape there cancels the scheduled task and stops all telemetry for the session.
            log("planting the health listener failed: $t")
            false
        }
    }

    override fun cancelListen() {
        val l = listener ?: return
        listener = null
        try {
            DeviceHealthManager.getInstance().removeDJIDeviceHealthInfoChangeListener(l)
        } catch (t: Throwable) {
            // Teardown must not throw over a listener the SDK has already forgotten.
            log("removing the health listener failed: $t")
        }
    }

    override fun current(): List<Warning>? = try {
        convert(DeviceHealthManager.getInstance().currentDJIDeviceHealthInfos)
    } catch (t: Throwable) {
        log("reading the current health list failed: $t")
        null
    }

    /**
     * DJI's list into ours, skipping anything unreadable rather than failing the whole delivery.
     *
     * A warning with no [Warning.code] is dropped: the code is the identity the whole diff is
     * keyed on, and an entry without one cannot be tracked, cleared, or searched for. It is logged
     * so the drop is not silent.
     */
    private fun convert(infos: List<DJIDeviceHealthInfo>?): List<Warning> {
        if (infos == null) return emptyList()
        val out = ArrayList<Warning>(infos.size)
        for (info in infos) {
            try {
                if (info == null) continue
                val code = info.informationCode()?.takeIf { it.isNotBlank() }
                if (code == null) {
                    log("skipping a health info with no informationCode (${info.warningLevel()})")
                    continue
                }
                val state = info.warningLevel()?.name
                out += Warning(
                    source = WarnSource.DEVICE_HEALTH,
                    code = code,
                    // DJI's own word, verbatim, alongside our translation of it — the record and
                    // the bus carry the first, the operator's sentence the second.
                    state = state ?: WarnLevel.UNKNOWN.name,
                    level = WarnLevel.ofName(state),
                    title = info.title(),
                    description = info.description(),
                    componentId = info.componentId(),
                    sensorIndex = info.sensorIndex(),
                )
            } catch (t: Throwable) {
                log("skipping an unreadable health info: $t")
            }
        }
        return out
    }
}
