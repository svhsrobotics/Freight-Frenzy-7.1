package org.firstinspires.ftc.teamcode.util;


import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


public class BroadcastHandler extends BroadcastReceiver {
    private final VoidLambda<Intent> lambda;

    public BroadcastHandler(String name, VoidLambda<Intent> lambda) {
        this.lambda = lambda;
        AppUtil.getDefContext().getApplicationContext().registerReceiver(this, new IntentFilter(name));
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        this.lambda.run(intent);
    }
}
