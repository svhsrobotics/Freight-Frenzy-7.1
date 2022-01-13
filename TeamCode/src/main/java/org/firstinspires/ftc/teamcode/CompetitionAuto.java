package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.Shared.Drive3;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;

@Autonomous
public class CompetitionAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);

        Configuration config = Configurator.load();
        if (config.target == null) {
            telemetry.log().add("WARNING: WAS NOT CALIBRATED");
            config.target = new HSVColor(0.0, 0.0, 0.0);
        }

        TeamElementDetector detector = new TeamElementDetector(config);
        webcam.setPipeline(detector);

        Robot robot = new Robot(hardwareMap);
        robot.initHardware();
        robot.arm.reset(); // This resets the encoder to 0

        Drive2 drive = new Drive2(robot,this);


        waitForStart();

        // Open the camera
        webcam.open();

        // Wait for the OpMode to start
        waitForStart();

        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        telemetry.addData("Position of the Team Element:", position);
        //Drive away from wall
        drive.navigationMonitorTicks(1.0/4, 0, -35, 10);

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
                robot.arm.goToPosition(Arm.HubPosition.BACKBOT);
                break;
            case CENTER:
                robot.arm.goToPosition(Arm.HubPosition.BACKMID);
                break;
            case RIGHT:
                robot.arm.goToPosition(Arm.HubPosition.BACKTOP);
                break;
        }


        //To Hub
        drive.navigationMonitorTicks(1/2, -5, -5, 10);
        robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
        //Sleeping so the collector has time to eject before stopping the servo
        sleep(2000);
        robot.arm.setCollectorMode(Arm.CollectorMode.Stop);
        //Drive over to Carousel
        drive.navigationMonitorTicks(1, 10, 0,10);
        //TODO: Include Code Using Sensor Data
        telemetry.update();

    }

}
