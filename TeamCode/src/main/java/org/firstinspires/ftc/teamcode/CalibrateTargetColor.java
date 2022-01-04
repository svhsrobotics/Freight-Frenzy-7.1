package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.Region;
import org.firstinspires.ftc.teamcode.vision.TeamElementCalibrator;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.opencv.core.Point;

import java.util.HashMap;

@TeleOp(name = "Calibrate Target Color", group = "Calibration")
public class CalibrateTargetColor extends LinearOpMode {

    @Override
    public void runOpMode() {
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        TeamElementCalibrator calibrator = new TeamElementCalibrator();
        webcam.setPipeline(calibrator);
        webcam.open();

        waitForStart();

        Configuration config = Configurator.load();

        // Add default values if they do not exist
        if (config.regions == null) {
            HashMap<TeamElementDetector.TeamElementPosition, Region> regions = new HashMap<>();
            regions.put(TeamElementDetector.TeamElementPosition.LEFT,
                    new Region(new Point(0.0, 0.0), 10, 10));
            config.regions = regions;
        }

        if (config.target == null) {
            config.target = new HSVColor(0.0,0.0,0.0);
        }

        if (config.threshold == null) {
            config.threshold = 0.0;
        }

        while (opModeIsActive()) {
            if (gamepad1.a) {
                config.target = new HSVColor(calibrator.getAnalysis());
                break; // Break the loop so we stop calibrating
            }
        }

    }
}
