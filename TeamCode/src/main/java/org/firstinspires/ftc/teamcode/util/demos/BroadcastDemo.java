package org.firstinspires.ftc.teamcode.util.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BroadcastHandler;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class BroadcastDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // You need to wrap all variables you want to be modifiable be the handlers in AtomicReferences
        AtomicReference<Boolean> stopped = new AtomicReference<>(false);
        AtomicReference<String> color = new AtomicReference<>("");

        // You can pass a single command to execute
        // Note the use of .set because they are AtomicReferences
        new BroadcastHandler("stop", intent -> stopped.set(true));

        // Or multiple enclosed in braces
        new BroadcastHandler("setColor", intent -> {
            if (intent.getStringExtra("color") != null) {
                color.set(intent.getStringExtra("color"));
            }
        });

        waitForStart();
        telemetry.log().add("Use command: adb shell am broadcast -a [handler name] --es [string name] \\\"[string content]\\\"");
        telemetry.update();
        while (opModeIsActive() && !stopped.get()) {
            // Note the use of .get because they are AtomicReferences
            telemetry.addData("Color", color.get());
            telemetry.update();
            sleep(50);
        }
    }
}
