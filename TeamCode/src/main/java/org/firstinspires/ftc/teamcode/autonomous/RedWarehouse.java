package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm.CollectorMode;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;

@Autonomous(name = "Red Warehouse", group = "Competition")
public class RedWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        lights.setPattern(BlinkinPattern.RED);

        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        Servo cap;
        CRServo rightCarousel;
        CRServo leftCarousel;
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
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        // Initialize the hardware
        robot.initHardware();
        // Reset the arm encoder to 0 to prevent some issues...
        robot.arm.reset();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot, this);
        //Drive3 drive = new Drive3(this);
        //drive.init();

        // Open the camera; also begins streaming the pipeline
        webcam.open();
        cap.setPosition(1);

        lights.setPattern(BlinkinPattern.LIGHT_CHASE_RED);

        // Wait for the OpMode to start
        // Make sure to do this after the camera is opened; otherwise "View Camera Stream" won't work
        waitForStart();

        while (!detector.isReady()) {
            sleep(50);
        }

        // As soon as we start, get the position
        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        // Print it to the telemetry
        telemetry.log().add("Position of the Team Element: " + position);
        telemetry.update();

        // Drive away from wall so the arm doesn't hit it.
        drive.navigationMonitorTicks(10, 0, -15, 10);
        drive.ceaseMotion();
        cap.setPosition(.5);

        // Raise the arm so it doesn't drag.
        robot.arm.setPositions(-1435, .38);

        drive.navigationMonitorTicks(10, 30, 0, 10);
        drive.ceaseMotion();

        switch (position) {
            case LEFT:
                // Back up a little, so we don't hit the hub
                drive.navigationMonitorTicks(10, 0, 5, 10);
                drive.ceaseMotion();
                robot.arm.goToBackPosition(HubPosition.BOTTOM);
                sleep(3000);
                drive.navigationMonitorTicks(10, 0, -1, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(CollectorMode.Stop);

                robot.arm.goToPosition(HubPosition.PARK);
                drive.navigationMonitorTicks(10, 0, 2, 10);
                break;
            case CENTER:
                // Back up a little, so we don't hit the hub
                drive.navigationMonitorTicks(10, 0, 6, 10);
                drive.ceaseMotion();
                robot.arm.goToBackPosition(HubPosition.MIDDLE);
                sleep(3000);
                drive.navigationMonitorTicks(10, 0, -1, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(CollectorMode.Stop);

                robot.arm.goToPosition(HubPosition.PARK);
                break;
            case RIGHT:
                robot.arm.goToBackPosition(HubPosition.TOP);
                sleep(3000);
                robot.arm.setCollectorMode(CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(CollectorMode.Stop);

                robot.arm.goToPosition(HubPosition.PARK);
                break;
        }

        drive.navigationMonitorTicksPhi(0, 10, 10, 90, 2.5);
        drive.ceaseMotion();
        drive.navigationMonitorTicksPhi(10, 5, 0, 90, 10);
        drive.navigationMonitorTicksPhi(40, 0, 70, 90, 10);
        drive.ceaseMotion();

        lights.setPattern(BlinkinPattern.RED);

        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
