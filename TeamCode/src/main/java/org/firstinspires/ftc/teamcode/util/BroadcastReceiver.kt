package org.firstinspires.ftc.teamcode.util

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter


class BroadcastReceiver(
        intentAction: String = "debug",
        private val receiver: (Intent) -> Unit
    ) : BroadcastReceiver() {

        init {
            Application.getContext().registerReceiver(this, IntentFilter(intentAction))
        }

        override fun onReceive(context: Context, intent: Intent) {
            receiver(intent)
        }
}