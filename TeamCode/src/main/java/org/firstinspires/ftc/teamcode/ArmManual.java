package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class ArmManual extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create robot from hardware map
        Robot robot = new Robot(hardwareMap);
        // Initialize the robot hardware
        robot.initHardware();
        // Initialize the drivetrain

        // Wait for the start button
        waitForStart();
        // Drive away from the wall
        while (opModeIsActive()) {
            robot.arm.arm.setMode(RunMode.RUN_USING_ENCODER);
            robot.arm.arm.setPower(Math.pow(gamepad2.right_stick_y, 3));
            telemetry.addData("ARM", robot.arm.arm.getCurrentPosition());

            if (gamepad2.a) {
                robot.arm.setWristPosition(gamepad2.left_stick_y);
            }
            telemetry.addData("WRIST", robot.arm.wrist.getPosition());

            telemetry.update();
        }
    }
}
