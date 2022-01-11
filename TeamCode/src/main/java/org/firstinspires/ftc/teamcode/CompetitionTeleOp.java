package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Competition", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {
    //private final String TAG = getClass().getName();
    DcMotor Arm = null;
    Servo Wrist = null;
    CRServo Collector = null;
    double pivotCollectorFactor = 0.17 / 2;
    double pivotCollectorDifference = (0.17 / 2) + 0.36;
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;


    @Override
    public void runOpMode() {
        //final Drive2 drive = new Drive2(this);

        //drive.init();

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Wrist = hardwareMap.get(Servo.class, "pivotCollector");
        Collector = hardwareMap.get(CRServo.class, "spinCollector");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");



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

           // leftFrontDrive.setPower((frontLeftPowerFactor * magLeft)*(frontLeftPowerFactor * magLeft));
           // rightFrontDrive.setPower(-(frontRightPowerFactor * magRight)*(frontRightPowerFactor * magRight));
            //leftBackDrive.setPower((backLeftPowerFactor * magLeft)*(backLeftPowerFactor * magLeft));
            //rightBackDrive.setPower(-(backRightPowerFactor * magRight)*(backRightPowerFactor * magRight));

            leftFrontDrive.setPower(-(frontLeftPowerFactor * magLeft));
            rightFrontDrive.setPower((frontRightPowerFactor * magRight));
            leftBackDrive.setPower(-(backLeftPowerFactor * magLeft));
            rightBackDrive.setPower((backRightPowerFactor * magRight));

            if (gamepad2.y) {
                if(gamepad2.dpad_down){
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.5);
                Wrist.setPosition(0.53);
            } else if (gamepad2.dpad_up){
                    Arm.setTargetPosition(-340);//-300
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.5);
                    Wrist.setPosition(0.5);
                }}
                else if (gamepad2.back) {
                GoToHubLevel(1);
            } else if (gamepad2.x) {
                GoToHubLevel(2);
            } else if (gamepad2.b) {
                GoToHubLevel(3);
            } else if (gamepad2.a) {
                GoToHubLevel(4);
            } else if (gamepad2.dpad_right) {
                Wrist.setPosition(-gamepad2.right_stick_y * pivotCollectorFactor + pivotCollectorDifference);
            } else if (gamepad2.dpad_up) {
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.setPower((gamepad2.right_trigger + (-gamepad2.left_trigger)) / 2);
            }else if (gamepad2.right_stick_y==1){
                    //manual driver control of arm
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setTargetPosition(Arm.getCurrentPosition()+100);
            }else if (gamepad2.right_stick_y==-1) {
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setTargetPosition(Arm.getCurrentPosition()-100);
            }else if (gamepad2.right_stick_x==1){
                    //manual driver control of wrist
                    Wrist.setPosition(Wrist.getPosition()-.01);
                    sleep(100);
            }else if (gamepad2.right_stick_x==-1){
                Wrist.setPosition(Wrist.getPosition()+.01);
                sleep(100);
            }
            Collector.setPower(-gamepad2.left_stick_y / 2);

            telemetry.addData("Ticks", Arm.getCurrentPosition());
            telemetry.addData("WristPos", Wrist.getPosition());
            telemetry.addData("Power", Collector.getPower());
            telemetry.addData("ArmPower", gamepad2.right_trigger + (-gamepad2.left_trigger));
            telemetry.addData("front right power ", ((float) Math.round(rightFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("front left power ", ((float) Math.round(leftFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("back right power ", ((float) Math.round(rightBackDrive.getPower() * 100)) / 100);
            telemetry.addData("back left power ", ((float) Math.round(leftBackDrive.getPower() * 100)) / 100);
            telemetry.addData("left joystick x", ((float) Math.round(gamepad1.left_stick_x * 100)) / 100);
            telemetry.addData("left joystick y", ((float) Math.round(-gamepad1.left_stick_y * 100)) / 100);
            telemetry.addData("magnitude left", ((float) Math.round(magLeft * 100)) / 100);
            telemetry.addData("thetaLeft", ((float) Math.round(thetaLeft / pi * 100)) / 100);

            telemetry.update();


        }

        // Only do this in simulator; real robot needs time to stop.
        //drive.ceaseMotion();
    }

    private void GoToHubLevel(int hubLevel) {

        if (hubLevel == 1) {
            Arm.setTargetPosition(-3720);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(1);
            Wrist.setPosition(0.38);
        }

        if (hubLevel == 2) {
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-4100);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.36);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                Arm.setTargetPosition(-2200);//-350
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.53);
                Arm.setPower(1);
            }
        }
        if (hubLevel == 3) {
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-5238);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.44);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                Arm.setTargetPosition(-1450);//-250
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.48);
                Arm.setPower(1);
            }
        }

        if (hubLevel == 4) {
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-6300);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.49);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                Arm.setTargetPosition(-430);//-250
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.43);
                Arm.setPower(1);
            }

        }
    }
}
