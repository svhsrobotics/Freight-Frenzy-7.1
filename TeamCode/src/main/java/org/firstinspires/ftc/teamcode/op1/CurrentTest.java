package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled
@TeleOp
public class CurrentTest extends LinearOpMode {

    public void runOpMode() {
        Robot robot1 = new Robot(hardwareMap);
        robot1.initHardware();

        Drive2 drive = new Drive2(robot1, this);
        //drive.init();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.navigationMonitorTicks(1 / 4, 0, 0, 10);
            }
        }


    }
}
