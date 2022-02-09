package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;

@Autonomous(name = "Competition Auto", group = "Competition")
public class CompetitionAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        Servo cap;
        cap = hardwareMap.get(Servo.class, "cap");
        // Load the configuration
        Configuration config = Configurator.load();
        if (config.target == null) {
            // This is an extra warning to make debugging errors a little easier
            telemetry.log().add("WARNING: WAS NOT CALIBRATED");
            // So we don't get a NullPointerException later on...
            config.target = new HSVColor(0.0, 0.0, 0.0);
        }

        // Create a detector pipeline from the config
        TeamElementDetector detector = new TeamElementDetector(config);
        // Tell the camera to start using the pipeline
        webcam.setPipeline(detector);
        // Create the robot from the hardware map
        Robot robot = new Robot(hardwareMap);
        // Initialize the hardware
        robot.initHardware();
        // Reset the arm encoder to 0 to prevent some issues...
        robot.arm.reset();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot,this);
        //Drive3 drive = new Drive3(this);
        //drive.init();

        // Open the camera; also begins streaming the pipeline
        webcam.open();
        cap.setPosition(0);

        // Wait for the OpMode to start
        // Make sure to do this after the camera is opened; otherwise "View Camera Stream" won't work
        cap.setPosition(0);
        waitForStart();

        while (!detector.isReady()) {
            sleep(50);
        }

        // As soon as we start, get the position
        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        // Print it to the telemetry
        telemetry.log().add("Position of the Team Element: " + position); telemetry.update();

        // Drive away from wall so the arm doesn't hit it.
        drive.navigationMonitorTicks(10, 0, -15, 10);
        drive.ceaseMotion();
        cap.setPosition(.5);

        // Raise the arm so it doesn't drag.
        robot.arm.setPositions(-1435, .52);
        drive.navigationMonitorTicks(10, 15, 0, 10);
        sleep(1000);
        drive.navigationMonitorTicks(2, 2, 0, 10); // NB: Need external version
        drive.ceaseMotion();
        sleep(10000);
        drive.navigationMonitorTicks(20, -15, 0, 10);
        if (position == TeamElementDetector.TeamElementPosition.RIGHT) {
            // Do an extra 8 inches to the left to get around the block- otherwise we plow it into the way
            drive.navigationMonitorTicks(20, 10, 0, 10);
        }

        drive.navigationMonitorTicks(20, 0, -70, 10);
        drive.ceaseMotion();
        sleep(1000);
        if (position == TeamElementDetector.TeamElementPosition.RIGHT) {
            drive.navigationMonitorTicks(15, -30, 0, 10);
        } else {
            drive.navigationMonitorTicks(15, -22, 0, 10);
        }
        drive.ceaseMotion();

        // Position to level mapping:
        //        []
        //    \___||___/  <-- RIGHT
        //        ||
        //   \____||____/  <-- CENTER
        //        ||
        //  \_____||_____/  <-- LEFT
        //


        // Position the arm to the correct level
        // Also need to get a few inches closer
        switch (position) {
            case LEFT:
                robot.arm.goToPosition(Arm.HubPosition.BOT);
                //robot.arm.setPositions(-398, .42);
                //robot.arm.setPositions(-369, .57);
                drive.navigationMonitorTicks(15, 0, 5, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(4000);
                drive.navigationMonitorTicks(15, 0, -11, 10); // Move back
                drive.ceaseMotion();
                break; // Break is *very* important
            case CENTER:
                robot.arm.goToPosition(Arm.HubPosition.MID);//FRONTLOAD
                //robot.arm.setPositions(-1157, .47);
                //robot.arm.setPositions(-1270, .67);
                drive.navigationMonitorTicks(15, 0, 5.75, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(4000);
                drive.navigationMonitorTicks(15, 0, -11.75, 10); // Move back
                drive.ceaseMotion();
                break;
            case RIGHT:
                robot.arm.goToPosition(Arm.HubPosition.TOP);
                //robot.arm.setPositions(-2180, .57);
                //robot.arm.setPositions(-2019, .71);
                drive.navigationMonitorTicks(15, 0, 9, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(4000);
                drive.navigationMonitorTicks(15, 0, -17, 10); // Move back
                drive.ceaseMotion();
                break;
        }

        robot.arm.setCollectorMode(Arm.CollectorMode.Stop);

        //robot.arm.setPositions(0, 1.0);

        drive.navigationMonitorTicks(15, -55, 0,10);
        drive.ceaseMotion();

        drive.navigationMonitorTicks(25, 0, 64, 10);
        drive.ceaseMotion();

        robot.arm.goToPosition(Arm.HubPosition.PARK);
        cap.setPosition(.5);
        //robot.arm.setPositions(-Arm.ARM_OFFSET, 0.25);

        // Drive to the alliance hub
        //drive.navigationMonitorTicks(1.0/2, -5, 0, 10);
        // Start ejecting the block
        //robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
        // Wait for it to eject (should tune this amount...)
        //sleep(2000);
        // Ok stop ejecting now
        //robot.arm.setCollectorMode(Arm.CollectorMode.Stop);

        // Drive to the carousel
        //drive.navigationMonitorTicks(1.0, 10, 0,10);

        // IDK what else needs to happen. Something about positioning for the carousel

        // Now we need to park, right?

        // Just wait for the OpMode to end... Very important so that things continue running lol.
        while (opModeIsActive()) {
            //Thread.sleep(2000);
            //telemetry.addData("Angle", robot.imu.getAngularOrientation());
            //telemetry.update();
            sleep(50);
        }
    }

}
