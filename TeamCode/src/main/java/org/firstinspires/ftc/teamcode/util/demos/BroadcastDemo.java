package org.firstinspires.ftc.teamcode.util.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BroadcastReceiver;

@Autonomous
public class BroadcastDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new BroadcastReceiver("debug", (intent) -> {
            String color = intent.getStringExtra("color");
            android.util.Log.i("TEST", color);
            return null;
        });
        waitForStart();
    }
}
