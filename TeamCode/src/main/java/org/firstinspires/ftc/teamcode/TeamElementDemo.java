package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.TeamElementCalibrator;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.vision.Webcam;
import org.openftc.easyopencv.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous
public class TeamElementDemo extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        
        // Setup the detector pipeline
        TeamElementDetector detector = new TeamElementDetector(CalibrateTargetColor.getCalibration());
        webcam.setPipeline(detector);

        // Open the camera
        webcam.open();

        // Wait for the OpMode to start
        waitForStart();

        // While the OpMode is running
        while (opModeIsActive())
        {
            // Log the result of our analysis
            telemetry.addData("Analysis", detector.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping
            // Normally, more processing would go here
            sleep(50);
        }
    }
}
