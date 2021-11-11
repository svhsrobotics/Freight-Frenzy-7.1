package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Shared.Drive2;

@TeleOp
public class navTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LM DT");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RM DT");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LR DT");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RR DT");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        Drive2 drive = new Drive2(this);
        drive.init();
        waitForStart();

        while (opModeIsActive()){
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

            leftFrontDrive.setPower((frontLeftPowerFactor * magLeft)*(frontLeftPowerFactor * magLeft));
            rightFrontDrive.setPower(-(frontRightPowerFactor * magRight)*(frontRightPowerFactor * magRight));
            leftBackDrive.setPower((backLeftPowerFactor * magLeft)*(backLeftPowerFactor * magLeft));
            rightBackDrive.setPower(-(backRightPowerFactor * magRight)*(backRightPowerFactor * magRight));
        }

    }
}
