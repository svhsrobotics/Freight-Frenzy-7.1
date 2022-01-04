package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.opencv.core.Scalar;

@Autonomous
public class CompetitionAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);

        Scalar target = getTargetColor();

        TeamElementDetector detector = new TeamElementDetector(target);
        webcam.setPipeline(detector);

        // Open the camera
        webcam.open();

        // Wait for the OpMode to start
        waitForStart();

        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        telemetry.addData("Position of the Team Element:", position);
        switch (position) {
            case LEFT:
                //
            case CENTER:
                //
            case RIGHT:
                //
        }

        telemetry.update();

    }

    public Scalar getTargetColor() {
        Configuration config = new Configuration();
        HSVColor hsv;

        if (config.target != null) {
            hsv = config.target;
        } else {
            telemetry.log().add("WARNING WARNING WARNING:");
            telemetry.log().add("TARGET COLOR WAS NOT CALIBRATED!!!");
            telemetry.log().add("Please run the Calibrate Target Color OpMode!");
            android.util.Log.w("TeamElementDemo", "Target Color was NOT CALIBRATED! Falling back to default");
            hsv = new HSVColor(0.0,0.0,0.0);
        }
        return hsv.toScalar();
    }

}
