package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.NeverStops;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.opencv.core.Scalar;

@Autonomous
@Disabled
public class CompetitionNoVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap);
        robot.initHardware();

        Drive2 drive = new Drive2(robot,this);
        //drive.init();

        Arm arm = robot.arm;


        // Wait for the OpMode to start
        waitForStart();
        //Drive away from wall
        drive.navigationMonitorTicks(1/4, 0, -5, 10, false);

        //To Hub
        drive.navigationMonitorTicks(1/2, -5, -5, 10, false);
        arm.setCollectorMode(Arm.CollectorMode.Eject);
        //Sleeping so the collector has time to eject before stopping the servo
        sleep(2000);
        arm.setCollectorMode(Arm.CollectorMode.Stop);
        //Drive over to Carousel
        drive.navigationMonitorTicks(1, 10, 0,10, false);
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
