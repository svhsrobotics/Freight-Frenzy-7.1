package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shared.Drive2;

@TeleOp(name = "Arm Demo", group = "Drive2")
public class ArmDemo extends LinearOpMode {
    private final String TAG = getClass().getName();
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

            if(gamepad2.a){
                Wrist.setPosition(0.36);
            }
            if (gamepad2.b){
                Wrist.setPosition(0.53);
            }
            if (gamepad2.x){
                Wrist.setPosition(0.5);
            }
            Arm.setPower(gamepad2.right_trigger + (-gamepad2.right_trigger));
            Collector.setPower(-gamepad2.right_stick_y);

            Wrist.setPosition(-gamepad2.left_stick_y * pivotCollectorFactor + pivotCollectorDifference);

            telemetry.addData("Ticks", Arm.getCurrentPosition());
            telemetry.addData("WristPos", Wrist.getPosition());
            telemetry.addData("Power", Collector.getPower());

        }

        // Only do this in simulator; real robot needs time to stop.
        //drive.ceaseMotion();
    }
}
