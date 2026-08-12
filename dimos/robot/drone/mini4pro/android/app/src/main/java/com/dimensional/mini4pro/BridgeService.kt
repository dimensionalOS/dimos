package com.dimensional.mini4pro

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.IBinder
import android.util.Log
import com.dimensional.mini4pro.mavlink.MavlinkLink
import com.dimensional.mini4pro.video.VideoRequest
import com.dimensional.mini4pro.zenoh.ZenohSettings

/**
 * Runs the MAVLink bridge as a foreground service.
 *
 * ## Why this is not optional
 *
 * The bridge used to live in [MainActivity]'s process with no service at all. That
 * works only while our screen is the visible one — which is exactly what a field
 * session is not. The operator watches the camera in DJI Fly, or the screen turns
 * off, our process becomes *cached*, and Android's app freezer suspends it. A held
 * `PARTIAL_WAKE_LOCK` does not prevent this: it stops the CPU sleeping, not the
 * process being frozen.
 *
 * Measured on 2026-07-25 from a real session's flight log, all within one live
 * process — the app was never killed or restarted:
 *
 * ```
 * t=  268.0s  silent 601.8s   phone unplugged and carried to the RC
 * t=  913.4s  silent  94.7s
 * t= 1013.0s  silent  19.2s
 * t= 1087.2s  silent   8.0s   aircraft connected, DJI Fly in front
 * ```
 *
 * 322 heartbeats went out where 1036 were due. QGroundControl reported that as
 * "Communication Lost", and only 5% of its own heartbeats reached us, because a
 * frozen process cannot receive either.
 *
 * A telemetry link that stops when the operator looks at the camera is not a
 * telemetry link. The persistent notification this service is obliged to show is a
 * feature rather than a cost: it is the operator's proof that the bridge is alive.
 *
 * ## Type
 *
 * `connectedDevice`, because that is what we are — a bridge to an aircraft attached
 * through the RC on USB. `dataSync` carries tighter runtime limits on recent Android
 * versions and would be a worse fit for something that must run for a whole flight.
 *
 * Battery-optimisation exemption is **still worth having on top of this** — HyperOS
 * is more aggressive than stock Android — but it is a device setting we cannot rely
 * on being present, whereas this is ours.
 */
class BridgeService : Service() {

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val host = intent?.getStringExtra(EXTRA_HOST).orEmpty()
        val port = intent?.getIntExtra(EXTRA_PORT, MavlinkLink.DEFAULT_GCS_PORT)
            ?: MavlinkLink.DEFAULT_GCS_PORT

        // Go foreground *before* any work: Android gives a narrow window after
        // startForegroundService() and kills the process if we miss it.
        startForeground(NOTIFICATION_ID, notification(host, port))

        if (host.isEmpty()) {
            Log.w(TAG, "no host supplied; stopping")
            stopSelf()
            return START_NOT_STICKY
        }

        // Resolved by the caller and carried whole, rather than re-derived here:
        // the service is also started from a redelivered intent it did not
        // compose, and "where does video go" must not have two answers.
        val video = intent?.let {
            VideoRequest.resolve(
                intentEnabled = it.getBooleanExtra(EXTRA_VIDEO, false),
                intentHost = it.getStringExtra(EXTRA_VIDEO_HOST),
                intentPort = it.getIntExtra(EXTRA_VIDEO_PORT, 0).takeIf { p -> p > 0 },
                // Falls back to the telemetry target, which is the point: the
                // relay carries :14550 and :5600 in one process.
                gcsHost = host,
            )
        }
        // Carried whole for the same reason as video, and with one difference that matters: the
        // Zenoh router is **not** derived from the GCS address. Video follows telemetry because
        // the relay carries both in one process; a Zenoh bus is a `zenohd` several consumers
        // attach to independently, and defaulting it to QGroundControl's laptop would dial a port
        // nothing listens on. See `ZenohSettings`.
        val zenoh = intent?.let {
            ZenohSettings.resolve(
                intentEnabled = it.getBooleanExtra(EXTRA_ZENOH, false),
                intentHost = it.getStringExtra(EXTRA_ZENOH_HOST),
                intentPort = it.getIntExtra(EXTRA_ZENOH_PORT, 0).takeIf { p -> p > 0 },
                intentPrefix = it.getStringExtra(EXTRA_ZENOH_PREFIX),
                intentVideo = it.getBooleanExtra(EXTRA_ZENOH_VIDEO, false),
                intentDetections = it.getBooleanExtra(EXTRA_ZENOH_DETECTIONS, false),
            )
        }
        Bridge.start(host, port, applicationContext, video, zenoh)

        // START_STICKY: if we are killed for memory pressure mid-flight, come back.
        // The intent is redelivered as null, which is why host/port are also in
        // SharedPreferences — see MainActivity's adb-free autostart.
        return START_STICKY
    }

    override fun onDestroy() {
        Bridge.stop()
        super.onDestroy()
    }

    private fun notification(host: String, port: Int): Notification {
        val manager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            // LOW, not DEFAULT: this must be visible and silent. A telemetry bridge
            // that pings is a telemetry bridge the operator mutes.
            val channel = NotificationChannel(
                CHANNEL_ID, "MAVLink bridge", NotificationManager.IMPORTANCE_LOW,
            ).apply {
                description = "Shown while the bridge is sending telemetry to a ground station."
                setShowBadge(false)
            }
            manager.createNotificationChannel(channel)
        }

        val open = PendingIntent.getActivity(
            this, 0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )

        val builder = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Notification.Builder(this, CHANNEL_ID)
        } else {
            @Suppress("DEPRECATION")
            Notification.Builder(this)
        }

        return builder
            .setContentTitle("MAVLink bridge running")
            .setContentText("Telemetry → $host:$port")
            .setSmallIcon(android.R.drawable.stat_sys_upload)
            .setContentIntent(open)
            .setOngoing(true)
            .build()
    }

    companion object {
        private const val TAG = "BridgeService"
        private const val CHANNEL_ID = "mini4pro.bridge"
        private const val NOTIFICATION_ID = 1
        const val EXTRA_HOST = "host"
        const val EXTRA_PORT = "port"
        const val EXTRA_VIDEO = "video"
        const val EXTRA_VIDEO_HOST = "videoHost"
        const val EXTRA_VIDEO_PORT = "videoPort"
        const val EXTRA_ZENOH = ZenohSettings.EXTRA_ENABLED
        const val EXTRA_ZENOH_HOST = ZenohSettings.EXTRA_HOST
        const val EXTRA_ZENOH_PORT = ZenohSettings.EXTRA_PORT
        const val EXTRA_ZENOH_PREFIX = ZenohSettings.EXTRA_PREFIX
        const val EXTRA_ZENOH_VIDEO = ZenohSettings.EXTRA_VIDEO
        const val EXTRA_ZENOH_DETECTIONS = ZenohSettings.EXTRA_DETECTIONS

        /**
         * Starts the bridge in its own foreground service. Safe to call repeatedly.
         *
         * Video rides in here rather than being started by the activity, and the
         * reason is the same one this service exists for at all: the operator
         * spends real sessions with DJI Fly in front or the screen off, and an
         * activity-scoped camera stream would be frozen by Android's app freezer
         * exactly as telemetry was on 2026-07-25. The foreground-service *type*
         * does not change — `connectedDevice` is still what we are. The frames
         * arrive over USB from the RC; nothing here touches Android's camera, so
         * no `camera` type and no `CAMERA` permission are involved.
         */
        fun start(
            context: Context,
            host: String,
            port: Int,
            video: VideoRequest.Plan? = null,
            zenoh: ZenohSettings.Plan? = null,
        ) {
            val intent = Intent(context, BridgeService::class.java)
                .putExtra(EXTRA_HOST, host)
                .putExtra(EXTRA_PORT, port)
                .putExtra(EXTRA_VIDEO, video?.enabled == true)
                .putExtra(EXTRA_VIDEO_HOST, video?.host)
                .putExtra(EXTRA_VIDEO_PORT, video?.port ?: 0)
                .putExtra(EXTRA_ZENOH, zenoh?.enabled == true)
                .putExtra(EXTRA_ZENOH_HOST, zenoh?.host)
                .putExtra(EXTRA_ZENOH_PORT, zenoh?.port ?: 0)
                .putExtra(EXTRA_ZENOH_PREFIX, zenoh?.prefix)
                // These two fields of the plan used to be dropped here. The activity resolved
                // `video = true`, this intent didn't carry it, and onStartCommand re-resolved
                // with the default — so the switch, the pref and `--ez zenohVideo` all agreed
                // and the bus still came up without the channel. Found on the first hardware
                // test, 2026-07-28, by the absence of the "video channel ON" warning in logcat.
                // `detections` then repeated the incident the same day: it was written against
                // the pre-fix base, and every plan field this intent carries has to be carried
                // *by name*, so a new field is a new chance to forget. Anything added to
                // `ZenohSettings.Plan` that `resolve()` folds from an intent extra MUST be
                // added here and in onStartCommand, or it silently becomes its default.
                .putExtra(EXTRA_ZENOH_VIDEO, zenoh?.video == true)
                .putExtra(EXTRA_ZENOH_DETECTIONS, zenoh?.detections == true)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                context.startForegroundService(intent)
            } else {
                context.startService(intent)
            }
        }

        fun stop(context: Context) {
            context.stopService(Intent(context, BridgeService::class.java))
        }
    }
}
