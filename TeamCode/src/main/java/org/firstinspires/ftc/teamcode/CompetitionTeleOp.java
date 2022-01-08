package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "Competition", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    Arm arm;
    Robot robot;

    @Override
    public void runOpMode() {

        //DcMotor arm = hardwareMap.get(DcMotor.class, "Arm");
        //Servo wrist = hardwareMap.get(Servo.class, "pivotCollector");
        //CRServo collector = hardwareMap.get(CRServo.class, "spinCollector");

        //this.arm = new Arm(arm, wrist, collector);

        //this.robot = new Robot(hardwareMap, logger);

        //logger.info("Button to level mapping:");
        logger.info("<tt>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Y]</tt>");
        logger.info("<tt>&nbsp;&nbsp;&nbsp;&nbsp;\\___|X|___/</tt>");
        logger.info("<tt>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;|</tt>");
        logger.info("<tt>&nbsp;&nbsp;&nbsp;\\____|B|____/</tt>");
        logger.info("<tt>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;|</tt>");
        logger.info("<tt>&nbsp;&nbsp;\\_____|A|_____/</tt>");
        logger.info("Special positions:");
        logger.info("Collect: &uarr;");
        logger.info("Starting position: &#10096; + &darr;");
        logger.warning("Test");
        telemetry.addData("Test", "test");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
            double magRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double thetaRight = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double magLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double thetaLeft = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double pi = Math.PI;

            if (thetaRight > 0 && thetaRight < pi / 2) {
                frontRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= -pi && thetaRight < -pi / 2) {
                frontRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight >= pi / 2 && thetaRight <= pi) {
                frontRightPowerFactor = 1;
            } else {
                frontRightPowerFactor = -1;
            }

            if (thetaLeft > 0 && thetaLeft < pi / 2) {
                backLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= -pi && thetaLeft < -pi / 2) {
                backLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= pi / 2 && thetaLeft <= pi) {
                backLeftPowerFactor = 1;
            } else {
                backLeftPowerFactor = -1;
            }

            if (thetaRight > -pi / 2 && thetaRight < 0) {
                backRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight > pi / 2 && thetaRight < pi) {
                backRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= 0 && thetaRight <= pi / 2) {
                backRightPowerFactor = 1;
            } else {
                backRightPowerFactor = -1;
            }

            if (thetaLeft > -pi / 2 && thetaLeft < 0) {
                frontLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft > pi / 2 && thetaLeft < pi) {
                frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= 0 && thetaLeft <= pi / 2) {
                frontLeftPowerFactor = 1;
            } else {
                frontLeftPowerFactor = -1;
            }

            robot.Drives.get(Robot.DrivePos.FRONT_LEFT).setPower((frontLeftPowerFactor * magLeft));
            robot.Drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(-(frontRightPowerFactor * magRight));
            robot.Drives.get(Robot.DrivePos.BACK_LEFT).setPower((backLeftPowerFactor * magLeft));
            robot.Drives.get(Robot.DrivePos.BACK_RIGHT).setPower(-(backRightPowerFactor * magRight));

            if (gamepad2.back && gamepad2.dpad_down) {
                this.arm.toLevel(Arm.HubLevel.Ground);
            } else if (gamepad2.back && gamepad2.dpad_up) {
                this.arm.toLevel(Arm.HubLevel.Ground, true);
            } else if (gamepad2.y) {
                this.arm.toLevel(Arm.HubLevel.Cap);
            } else if (gamepad2.x) {
                this.arm.toLevel(Arm.HubLevel.Top);
            } else if (gamepad2.b) {
                this.arm.toLevel(Arm.HubLevel.Middle);
            } else if (gamepad2.a) {
                this.arm.toLevel(Arm.HubLevel.Bottom);
            } else if (gamepad2.dpad_up) {
                this.arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.arm.arm.setPower((gamepad2.right_trigger - gamepad2.left_trigger) / 2);
            } else if (gamepad2.dpad_right) {
                double pivotCollectorFactor = 0.17 / 2;
                double pivotCollectorDifference = (0.17 / 2) + 0.36;
                this.arm.wrist.setPosition(-gamepad2.right_stick_y * pivotCollectorFactor + pivotCollectorDifference);
            }

            this.arm.collector.setPower(-gamepad2.left_stick_y / 2);
        }
    }
}
