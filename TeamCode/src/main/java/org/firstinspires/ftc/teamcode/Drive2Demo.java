package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shared.Drive2;

@TeleOp(name = "Drive 2 Demo", group = "Drive2")
public class Drive2Demo extends LinearOpMode {
    private final String TAG = getClass().getName();

    @Override
    public void runOpMode() {
        final Drive2 drive = new Drive2(this);
        drive.init();

        waitForStart();

        while (opModeIsActive()) {
            drive.check_and_set_drive(.5, -24, -24, 10);
            drive.check_and_set_drive(.5, 0, 24, 10);
            drive.check_and_set_drive(.5, 24, -24, 10);
            drive.check_and_set_drive(.5, 0, 24, 10);
        }

        // Only do this in simulator; real robot needs time to stop.
        drive.ceaseMotion();
    }
}
