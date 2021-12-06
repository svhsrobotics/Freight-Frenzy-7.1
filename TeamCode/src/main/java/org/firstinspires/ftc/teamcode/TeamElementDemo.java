package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.openftc.easyopencv.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous
public class TeamElementDemo extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        // TODO: Figure out what that camera monitor thing I removed did...

        // Get the camera from the hardware map
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Create a new pipeline object
        TeamElementDetector pipeline = new TeamElementDetector();
        // Tell the camera to use the pipeline
        webcam.setPipeline(pipeline);

        // Open the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Start streaming
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //TODO: Handle errors opening the camera
            }
        });

        // Wait for the OpMode to start
        waitForStart();

        // While the OpMode is running
        while (opModeIsActive())
        {
            // Log the result of our analysis
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping
            // Normally, more processing would go here
            sleep(50);
        }
    }
}
