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
            if (gamepad2.y) {
                Arm.setPower((gamepad2.right_trigger + (-gamepad2.left_trigger)) / 2);
            }

            if(gamepad2.a){
                Wrist.setPosition(0.5);
            }
            else if (gamepad2.b){
                Wrist.setPosition(0.53);
            }
            else if (gamepad2.x){
                GoToHubLevel(3);
            } else if (gamepad2.y){
                Wrist.setPosition(-gamepad2.left_stick_y * pivotCollectorFactor + pivotCollectorDifference);
            }
            Collector.setPower(-gamepad2.right_stick_y);

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
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (hubLevel == 3){
            Arm.setTargetPosition(-4444);
            Arm.setPower(1);
            Wrist.setPosition(0.36);

        }
        /* if (hubLevel == 2){
        Arm.setTargetPosition();
        Wrist.setPosition();
        Arm.setPower(0.5);
        }
         */
        /* if (hubLevel == 4){
        Arm.setTargetPosition();
        Wrist.setPosition();
        Arm.setPower(0.5);
        }
         */
        /* if (hubLevel == 1){
        Arm.setTargetPosition();
        Wrist.setPosition();
        Arm.setPower(0.5);
        }
         */
    }
}
