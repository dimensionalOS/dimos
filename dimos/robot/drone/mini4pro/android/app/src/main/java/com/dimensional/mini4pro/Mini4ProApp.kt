package com.dimensional.mini4pro

import android.app.Application
import android.content.Context

class Mini4ProApp : Application() {

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        // MSDK's native loader. Must run before anything touches the SDK, and
        // must be in attachBaseContext — not onCreate.
        com.cySdkyc.clx.Helper.install(this)
    }

    override fun onCreate() {
        super.onCreate()
        Msdk.init(this)
    }
}
