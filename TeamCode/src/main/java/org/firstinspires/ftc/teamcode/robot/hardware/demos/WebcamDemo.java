package org.firstinspires.ftc.teamcode.robot.hardware.demos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;

@TeleOp(name = "Webcam Demo", group = "Hardware Demos")
public class WebcamDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the webcam device from the hardware map
        // "Webcam 1" is the name of the camera, and may need to be changed depending on your config
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);

        // Initialize your pipeline, see TeamElementDetectorDemo for a better example
        TeamElementDetector detector = new TeamElementDetector(new HSVColor(0.0, 0.0, 0.0).toScalar());

        // Tell the camera to use that pipeline
        webcam.setPipeline(detector);

        // Open the camera device
        webcam.open();

        // You can now use the pipeline, see the TeamElementDetectorDemo for details

        // If we want to switch pipelines, we would do:
        //webcam.setPipeline(newpipeline);

        // If we want to save processing and stop processing *any* pipeline:
        webcam.pause();

        // To begin processing again:
        webcam.resume();

        // If we want to use Vuforia, we need to close the device; use open (from above) to re-open it.
        webcam.close();
    }
}
