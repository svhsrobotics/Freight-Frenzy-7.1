package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import android.app.Application
import android.content.Context

class Application {
    companion object {
        @SuppressLint("PrivateApi")
        fun getApplication(): Application {
            return Class.forName("android.app.ActivityThread")
                .getMethod("currentApplication").invoke(null) as Application
        }

        fun getContext(): Context {
            return getApplication().applicationContext
        }
    }
}