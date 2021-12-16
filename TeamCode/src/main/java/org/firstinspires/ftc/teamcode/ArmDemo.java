package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shared.Drive2;

@TeleOp(name = "Arm Demo", group = "Drive2")
public class ArmDemo extends LinearOpMode {
    //private final String TAG = getClass().getName();
    DcMotor Arm = null;
    Servo Wrist = null;
    CRServo Collector = null;
    double pivotCollectorFactor = 0.17/2;
    double pivotCollectorDifference = (0.17/2)+0.36;


    @Override
    public void runOpMode() {
        //final Drive2 drive = new Drive2(this);

        //drive.init();

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Wrist = hardwareMap.get(Servo.class, "pivotCollector");
        Collector = hardwareMap.get(CRServo.class, "spinCollector");


        waitForStart();


        while (opModeIsActive()) {

            if(gamepad2.back) {
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.5);
                Wrist.setPosition(0.53);
            } else if(gamepad2.y){
                GoToHubLevel(1);
            } else if (gamepad2.x){
                GoToHubLevel(2);
            } else if (gamepad2.b){
                GoToHubLevel(3);
            } else if (gamepad2.a) {
                GoToHubLevel(4);
            } else if (gamepad2.dpad_right){
                Wrist.setPosition(-gamepad2.right_stick_y * pivotCollectorFactor + pivotCollectorDifference);
            } else if (gamepad2.dpad_up) {
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.setPower((gamepad2.right_trigger + (-gamepad2.left_trigger)) / 2);
            }

            Collector.setPower(-gamepad2.left_stick_y / 2);

            telemetry.addData("Ticks", Arm.getCurrentPosition());
            telemetry.addData("WristPos", Wrist.getPosition());
            telemetry.addData("Power", Collector.getPower());
            telemetry.addData("ArmPower", gamepad2.right_trigger + (-gamepad2.left_trigger));
            telemetry.update();




        }

        // Only do this in simulator; real robot needs time to stop.
        //drive.ceaseMotion();
    }
    private void GoToHubLevel (int hubLevel) {

        /* if (hubLevel == 1){
            Arm.setTargetPosition();
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);
            Wrist.setPosition();
            } */

        if (hubLevel == 2){
            if(gamepad2.dpad_down) {
                Arm.setTargetPosition(-4100);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.36);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up){
                Arm.setTargetPosition(-2200);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(0.53);
                Arm.setPower(1);
            }
        }

        if (hubLevel == 3){
        Arm.setTargetPosition(-5238);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wrist.setPosition(0.44);
        Arm.setPower(1);
        }

        if (hubLevel == 4){
        Arm.setTargetPosition(-6300);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wrist.setPosition(0.49);
        Arm.setPower(1);
        }

    }
}
