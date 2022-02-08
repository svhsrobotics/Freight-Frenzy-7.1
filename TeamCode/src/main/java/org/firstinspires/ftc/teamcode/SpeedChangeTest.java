package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.util.NeverStops;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;

@Autonomous(name = "SpeedChangeTest", group = "Calibration")
public class SpeedChangeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Create the robot from the hardware map
        Robot robot = new Robot(hardwareMap);
        // Initialize the hardware
        robot.initDrives();
        robot.initIMU();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot, this);
        //Drive3 drive = new Drive3(this);
        //drive.init();


        // Wait for the OpMode to start
        waitForStart();

        drive.navigationMonitorTicks(20, 0, 40, 30, new NeverStops());
       // drive.navigationMonitorTicks(10, 0, 10, 30);
        drive.ceaseMotion();
    }
}