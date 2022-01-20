package org.firstinspires.ftc.teamcode.util.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous
@Disabled
public class LoggerDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        Logger logger = new Logger(telemetry);
        // Uncomment for production mode; doesn't print debug messages to telemetry
        //Logger logger = new Logger(telemetry, false);

        logger.debug("This is a debug message.");
        logger.info("This is an info message.");
        logger.warning("This is a warning message.");
        logger.error("This is an error message.");

        waitForStart();
    }
}
