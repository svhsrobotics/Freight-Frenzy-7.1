package org.firstinspires.ftc.teamcode.util.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BroadcastHandler;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class BroadcastDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        Logger logger = new Logger(telemetry);

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

        new BroadcastHandler("log", intent -> {
            String msg = intent.getStringExtra("msg");
            if (msg != null) {
                String level = intent.getStringExtra("level");
                if (level != null) {
                    switch (level) {
                        case "info":
                            logger.info(msg);
                            break;
                        case "warning":
                            logger.warning(msg);
                            break;
                        case "error":
                            logger.error(msg);
                            break;
                        case "debug":
                            logger.debug(msg);
                            break;
                    }
                } else {
                    logger.info(msg);
                }
            } else {
                logger.debug("Log was called without parameters.");
            }
        });

        logger.info("Use command: <u>adb shell am broadcast -a <i>[handler name]</i> --es <i>[parameter name] \\\"[parameter value]\\\"</i></u>");
        logger.info("  to send messages to handlers. Try passing something to the <i>color</i> parameter of <i>setColor</i> to see it appear above.");

        waitForStart();

        while (opModeIsActive() && !stopped.get()) {
            // Note the use of .get because they are AtomicReferences
            telemetry.addData("Color", color.get());
            telemetry.update();
            sleep(50);
        }
    }
}
