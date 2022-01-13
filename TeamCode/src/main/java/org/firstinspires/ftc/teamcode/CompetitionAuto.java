package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.opencv.core.Scalar;

@Autonomous
public class CompetitionAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);

        Scalar target = getTargetColor();

        TeamElementDetector detector = new TeamElementDetector(target);
        webcam.setPipeline(detector);

        Robot robot = new Robot(hardwareMap);
        robot.initHardware();

        Drive2 drive = new Drive2(robot,this);

        Arm arm = robot.arm;

        // Open the camera
        webcam.open();

        // Wait for the OpMode to start
        waitForStart();

        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        telemetry.addData("Position of the Team Element:", position);
        //Drive away from wall
        drive.navigationMonitorTicks(1/4, 0, -5, 10);

        //
        // Position to level mapping:
        //        []
        //    \___||___/  <-- RIGHT
        //        ||
        //   \____||____/  <-- CENTER
        //        ||
        //  \_____||_____/  <-- LEFT
        //
        switch (position) {
            case LEFT:
                arm.toLevel(Arm.HubLevel.Bottom);
                break;
            case CENTER:
                arm.toLevel(Arm.HubLevel.Middle);
                break;
            case RIGHT:
                arm.toLevel(Arm.HubLevel.Top);
                break;
        }


        //To Hub
        drive.navigationMonitorTicks(1/2, -5, -5, 10);
        arm.setCollectorMode(Arm.CollectorMode.Eject);
        //Sleeping so the collector has time to eject before stopping the servo
        sleep(2000);
        arm.setCollectorMode(Arm.CollectorMode.Stop);
        //Drive over to Carousel
        drive.navigationMonitorTicks(1, 10, 0,10);
        //TODO: Include Code Using Sensor Data
        telemetry.update();

    }

    public Scalar getTargetColor() {
        Configuration config = new Configuration();
        HSVColor hsv;

      if (config.target != null) {
            hsv = config.target;
        } else {
            telemetry.log().add("WARNING WARNING WARNING:");
            telemetry.log().add("TARGET COLOR WAS NOT CALIBRATED!!!");
            telemetry.log().add("Please run the Calibrate Target Color OpMode!");
            android.util.Log.w("TeamElementDemo", "Target Color was NOT CALIBRATED! Falling back to default");
            hsv = new HSVColor(0.0,0.0,0.0);
        }
        return hsv.toScalar();
    }

}
