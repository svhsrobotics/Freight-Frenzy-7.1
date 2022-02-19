package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled
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

        drive.navigationMonitorTicks(20, 0, 40, 30);
       // drive.navigationMonitorTicks(10, 0, 10, 30);
        drive.ceaseMotion();
    }
}