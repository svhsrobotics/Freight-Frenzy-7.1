package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.navigation.CarouselLocalizer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Timeout;

@Autonomous
public class CarouselTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create robot from hardware map
        Robot robot = new Robot(hardwareMap);
        // Initialize the robot hardware
        robot.initHardware();
        // Initialize the drivetrain
        Drive2 drive = new Drive2(robot, this);

        robot.arm.setWristPosition(0.71);
        // Wait for the start button
        waitForStart();
        // Drive away from the wall
        drive.navigationMonitorTicks(0.50, 20, -10, 10);
        // Stop moving
        drive.ceaseMotion();

        long nanos = System.nanoTime();
        // Move to the left until we encounter the carousel
        drive.navigationMonitorExternal(0.125, 8, 8, 10000, () -> {
            // Don't run until a second has passed
            if (System.nanoTime() - nanos > 1000000000) {
                // If the acceleration is greater than 1.0, stop moving
                return robot.imu.getLinearAcceleration().xAccel > 1.0;
            }
            return false;
        });
        drive.navigationMonitorTicks(0.125, 0.10, 0.10, 10000);
        // Stop moving
        drive.ceaseMotion();
    }
}
