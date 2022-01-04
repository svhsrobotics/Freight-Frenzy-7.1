package org.firstinspires.ftc.teamcode.vision.demos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;


@Autonomous
public class TeamElementDemo extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        Configuration config = Configurator.load();

        HSVColor target = config.target;

        if (target == null) {
            telemetry.log().add("WARNING WARNING WARNING:");
            telemetry.log().add("TARGET COLOR WAS NOT CALIBRATED!!!");
            telemetry.log().add("Please run the Calibrate Target Color OpMode!");
            android.util.Log.w("TeamElementDemo", "Target Color was NOT CALIBRATED! Falling back to default");
            target = new HSVColor(0.0,0.0,0.0);
            // Save that to the file permanently, so we don't get the warning next time?
            //config.target = target;
            //Configurator.save(config);
        }

        // Setup the detector pipeline
        TeamElementDetector detector = new TeamElementDetector(target.toScalar());
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
