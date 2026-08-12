package com.dimensional.mini4pro

import android.content.Context
import android.util.Log
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.network.DJINetworkManager
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Owns MSDK lifecycle: init -> registerApp -> product connect.
 *
 * Registration is an *online* activation of the App Key against DJI's servers,
 * so a first run needs internet even though the aircraft link is a USB cable.
 * Once activated it is cached, hence the network listener that retries.
 */
object Msdk {

    private const val TAG = "Msdk"

    data class State(
        val initEvent: DJISDKInitEvent? = null,
        val registered: Boolean = false,
        val registerError: IDJIError? = null,
        val productConnected: Boolean = false,
        val productId: Int = -1,
        /** Fly-safe database download, (current, total) bytes. */
        val dbProgress: Pair<Long, Long>? = null,
    )

    private val _state = MutableStateFlow(State())
    val state: StateFlow<State> = _state.asStateFlow()

    private var initialised = false

    @Volatile
    private var initComplete = false

    fun init(appContext: Context) {
        if (initialised) return
        initialised = true

        SDKManager.getInstance().init(appContext, object : SDKManagerCallback {
            override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                Log.i(TAG, "init: $event ($totalProcess)")
                _state.value = _state.value.copy(initEvent = event)
                // registerApp() must not be called before INITIALIZE_COMPLETE.
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    initComplete = true
                    SDKManager.getInstance().registerApp()
                }
            }

            override fun onRegisterSuccess() {
                Log.i(TAG, "registered")
                _state.value = _state.value.copy(registered = true, registerError = null)
            }

            override fun onRegisterFailure(error: IDJIError) {
                Log.e(TAG, "register failed: $error")
                _state.value = _state.value.copy(registered = false, registerError = error)
            }

            override fun onProductConnect(productId: Int) {
                Log.i(TAG, "product connected: $productId")
                _state.value = _state.value.copy(productConnected = true, productId = productId)
            }

            override fun onProductDisconnect(productId: Int) {
                Log.i(TAG, "product disconnected: $productId")
                _state.value = _state.value.copy(productConnected = false, productId = productId)
            }

            override fun onProductChanged(productId: Int) {
                Log.i(TAG, "product changed: $productId")
                _state.value = _state.value.copy(productId = productId)
            }

            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                _state.value = _state.value.copy(dbProgress = current to total)
            }
        })

        DJINetworkManager.getInstance().addNetworkStatusListener { available ->
            // The initComplete guard matters: this listener fires during init, and
            // calling registerApp() before INITIALIZE_COMPLETE wedges registration.
            if (initComplete && available && !SDKManager.getInstance().isRegistered) {
                Log.i(TAG, "network back, retrying registration")
                SDKManager.getInstance().registerApp()
            }
        }
    }
}
