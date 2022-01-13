package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Shared.Drive3;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;

@Autonomous
@Disabled
public class RedTeamNav extends LinearOpMode {
    @Override
    public void runOpMode(){
        Drive3 drive = new Drive3(this);
        drive.init();

        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        Configuration config = new Configuration();

        HSVColor hsv = null;

        if (config.target == null) {
            telemetry.log().add("WARNING WARNING WARNING:");
            telemetry.log().add("TARGET COLOR WAS NOT CALIBRATED!!!");
            telemetry.log().add("Please run the Calibrate Target Color OpMode!");
            android.util.Log.w("TeamElementDemo", "Target Color was NOT CALIBRATED! Falling back to default");
            config.target = new HSVColor(0.0,0.0,0.0);
        }
        // Wait for the OpMode to start
        waitForStart();

        // Open the camera
        webcam.open();

        int votes = 0;
        int rightvotes = 0;
        int leftvotes = 0;
        int centervotes = 0;

        DistanceSensor distanceLeftFront = hardwareMap.get(DistanceSensor.class,"DistanceLF");
        DistanceSensor distanceLeftSide = hardwareMap.get(DistanceSensor.class,"DistanceLS");
        DistanceSensor distanceRightFront = hardwareMap.get(DistanceSensor.class,"DistanceRF");
        DistanceSensor distanceRightSide = hardwareMap.get(DistanceSensor.class,"DistanceRS");

        double DistanceLF = 0;
        double DistanceLS =0;
        double DistanceRF = 0;
        double DistanceRS =0;

        TeamElementDetector detector = new TeamElementDetector(config);
/* wait to include information for json file
ask james in meeting today about team element detector and its proficiency
 */

        Double time = getRuntime();
        // While the less there is less than 1000 votes
        while (votes < 500 && opModeIsActive()) {
            // Log the result of our analysis
            telemetry.addData("Analysis", detector.getAnalysis());

            if (detector.getAnalysis() == TeamElementDetector.TeamElementPosition.RIGHT) {
                rightvotes++;
            } else if (detector.getAnalysis() == TeamElementDetector.TeamElementPosition.CENTER) {
                centervotes++;
            } else {
                leftvotes++;
            }
            // Don't burn CPU cycles busy-looping
            // Normally, more processing would go here

            votes++;
            Double elapsed = getRuntime() - time;
            telemetry.addData("Right Votes", rightvotes);
            telemetry.addData("Left Votes", leftvotes);
            telemetry.addData("Center Votes", centervotes);
            telemetry.addData("Total Votes", votes);
            telemetry.addData("Time", elapsed);
            telemetry.update();
            sleep(10);
        }

        if (leftvotes >= centervotes && leftvotes >= rightvotes) {
            telemetry.log().add("Decided left");
        } else if (centervotes >= leftvotes && centervotes >= rightvotes) {
            telemetry.log().add("Decided right");
        } else {
            telemetry.log().add("Decided center");
        }

        telemetry.update();

        sleep(10000);

/*
double TargetLF= (value from calibration in file)
double TargetRS= (value from calibration in file)
 */
        //arbitrarily made, find correct values to drive in order to be close to wall
        drive.vroomVroomMonitorTicks(1/2, 12,-5,10);
        /*
        //if the distance is more than a tenth of an inch away from target, it will complete this
        while((DistanceLF-TargetLF)>.1){
        drive.vroomVroomMonitorTicks(1/4, 0, .1, 2)
        }
        while((DistanceRS-TargetRS)>.1){
        drive.vroomVroomMonitorTicks(1/4, .1, 0, 2)
        }
        spin the servo with luke's code

         */
        //arbitrarily made, find correct values to drive in order to be in front of
        drive.vroomVroomMonitorTicks(1/2, -36,0,10);
        //extend arm to release the block
    }
    }
