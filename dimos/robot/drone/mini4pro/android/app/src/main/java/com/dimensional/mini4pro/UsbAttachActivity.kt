package com.dimensional.mini4pro

import android.app.Activity
import android.content.Intent
import android.os.Bundle

/**
 * Trampoline for USB_ACCESSORY_ATTACHED: plugging the RC-N2 in brings up the app.
 */
class UsbAttachActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        startActivity(
            Intent(this, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK)
        )
        finish()
    }
}
