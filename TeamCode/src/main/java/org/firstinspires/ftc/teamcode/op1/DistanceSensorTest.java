package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode(){

        DistanceSensor distanceLeft = hardwareMap.get(DistanceSensor.class,"distance left");
        ColorRangeSensor distanceright = hardwareMap.get(ColorRangeSensor.class, "distance right");

        double DR = 0;
        double DL =0;

        waitForStart();
       while (opModeIsActive()) {
           DL = distanceLeft.getDistance(DistanceUnit.INCH);
           telemetry.addData("Distance on the left side:", DL);
           DR = distanceright.getDistance(DistanceUnit.INCH);
           telemetry.addData("Distance on the right side", DR);
           telemetry.update();
       }
    }
}
