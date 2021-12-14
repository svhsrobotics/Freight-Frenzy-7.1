package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.vision.Calibration;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.Region;
import org.firstinspires.ftc.teamcode.vision.TeamElementCalibrator;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;
import org.firstinspires.ftc.teamcode.vision.Webcam;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Calibrate Target Color", group = "Calibration")
public class CalibrateTargetColor extends LinearOpMode {

    @Override
    public void runOpMode() {
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        TeamElementCalibrator calibrator = new TeamElementCalibrator();
        webcam.setPipeline(calibrator);
        webcam.open();

        waitForStart();

        Configuration config = new Configuration();

        // Add default values if they do not exist
        if (config.get("regions") == null) {
            HashMap<TeamElementDetector.TeamElementPosition, Region> regions = new HashMap<>();
            regions.put(TeamElementDetector.TeamElementPosition.LEFT,
                    new Region(new Point(0.0, 0.0), 10, 10));
            config.set("regions", regions);
        }

        if (config.get("target") == null) {
            HSVColor target = new HSVColor(0.0,0.0,0.0);
            config.set("target", target);
        }

        if (config.get("threshold") == null) {
            Double threshold = 0.0;
            config.set("threshold", threshold);
        }

        while (opModeIsActive()) {
            if (gamepad1.a) {
                HSVColor target = new HSVColor(calibrator.getAnalysis());
                config.set("target", target);
            }
        }

    }
}
