package org.firstinspires.ftc.teamcode.util;


import android.annotation.SuppressLint;
import android.app.Application;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


public class BroadcastHandler extends BroadcastReceiver {
    private final Lambda<Intent> lambda;

    public BroadcastHandler(String name, Lambda<Intent> lambda) {
        this.lambda = lambda;
        AppUtil.getDefContext().getApplicationContext().registerReceiver(this, new IntentFilter(name));
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        this.lambda.execute(intent);
    }
}
