package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode(){

        DistanceSensor distanceLeft = hardwareMap.get(DistanceSensor.class,"distance left");
        DistanceSensor distanceright = hardwareMap.get(DistanceSensor.class, "distance right");

        double DR = 0;
        double DL =0;

        //drive code from Ultimate Goal
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        waitForStart();
       while (opModeIsActive()) {
           DL = distanceLeft.getDistance(DistanceUnit.INCH);
           telemetry.addData("Distance on the left side:", DL);
           DR = distanceright.getDistance(DistanceUnit.INCH);
           telemetry.addData("Distance on the right side", DR);
           telemetry.update();

           //from Ultimate Goal
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
           //end ultimate goal
       }
    }
}
