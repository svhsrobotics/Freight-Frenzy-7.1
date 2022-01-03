package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Autonomous Demo", group = "Autonomous")
public class ArmAutonomousDemo extends LinearOpMode {
    //private final String TAG = getClass().getName();
    DcMotor Arm = null;
    Servo Wrist = null;
    CRServo Collector = null;
    int highLevel = -4444;


    @Override
    public void runOpMode() {
        //final Drive2 drive = new Drive2(this);

        //drive.init();

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Wrist = hardwareMap.get(Servo.class, "pivotCollector");
        Collector = hardwareMap.get(CRServo.class, "spinCollector");
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        while (opModeIsActive()) {

        // holder for visual input

            Arm.setTargetPosition(100);
            Arm.setPower(0.5);
            Wrist.setPosition(50);
            sleep(1000);
            Collector.setPower(-0.5);
            sleep(1000);
            Collector.setPower(0);


        }


    }
}
