package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.TeamElementCalibrator;
import org.firstinspires.ftc.teamcode.vision.Webcam;
import org.opencv.core.Scalar;

import java.io.File;

@TeleOp(name = "Calibrate Target Color", group = "Calibration")
public class CalibrateTargetColor extends LinearOpMode {

    @Override
    public void runOpMode() {
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        TeamElementCalibrator calibrator = new TeamElementCalibrator();
        webcam.setPipeline(calibrator);
        webcam.open();

        Gamepad gamepad = hardwareMap.get(Gamepad.class, "gamepad1");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad.a) {
                saveCalibration(calibrator.getAnalysis());
            }
        }
    }

    public void saveCalibration(Scalar color) {
        String filename = "calibration.txt";

        File file = AppUtil.getInstance().getSettingsFile(filename);

        ReadWriteFile.writeFile(file, color.toString());

    }
}
