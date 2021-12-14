package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCvTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Note: Camera monitor (viewer for camera on control hub HDMI has been removed).
        // Get the camera from the hardware map
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Create a new pipeline object
        TeamElementDetector pipeline = new TeamElementDetector(new Scalar(0.0,0.0,0.0));
        // Tell the camera to use the pipeline
        webcam.setPipeline(pipeline);

        // Open the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //TODO: Handle errors opening the camera
            }
        });

        int votes = 0;
        int rightvotes = 0;
        int leftvotes = 0;
        int centervotes = 0;

        // Wait for the OpMode to start
        waitForStart();
        //Whie Op Mode is Active
        while (opModeIsActive()) {
            // While the less there is less than 1000 votes
            while (votes < 500) {
                // Log the result of our analysis
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.update();
                //if analysis returns right
                if (pipeline.getAnalysis() == TeamElementDetector.TeamElementPosition.RIGHT) {
                    rightvotes++;
                    votes++;
                    telemetry.addData("Right Votes", rightvotes);
                    telemetry.addData("Left Votes", leftvotes);
                    telemetry.addData("Center Votes", centervotes);
                    telemetry.addData("Total Votes", votes);
                    //if analysis returns left
                } else if (pipeline.getAnalysis() == TeamElementDetector.TeamElementPosition.LEFT) {
                    leftvotes++;
                    votes++;
                    telemetry.addData("Left Votes", leftvotes);
                    telemetry.addData("Right Votes", rightvotes);
                    telemetry.addData("Center Votes", centervotes);
                    telemetry.addData("Total Votes", votes);
                    //if analysis returns center
                } else if (pipeline.getAnalysis() == TeamElementDetector.TeamElementPosition.CENTER) {
                    centervotes++;
                    votes++;
                    telemetry.addData("Left Votes", leftvotes);
                    telemetry.addData("Right Votes", rightvotes);
                    telemetry.addData("Center Votes", centervotes);
                    telemetry.addData("Total Votes", votes);
                }
                // Don't burn CPU cycles busy-looping
                // Normally, more processing would go here

                sleep(50);
            }

            //Decides on Protocol Based on if the votes are enough of a percentage of total
            if (rightvotes / votes > .6) {
                telemetry.addLine("Decision: Right");
            } else if (leftvotes / votes > .6) {
                telemetry.addLine("Decision: Left");
            } else if (centervotes / votes > .6) {
                telemetry.addLine("Decision: Center");
            }
            sleep(10000);
        }
    }
}
