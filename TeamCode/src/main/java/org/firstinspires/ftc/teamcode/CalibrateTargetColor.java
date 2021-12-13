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

    static final String FILENAME = "calibrateTargetColor.txt";

    @Override
    public void runOpMode() {
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        TeamElementCalibrator calibrator = new TeamElementCalibrator();
        webcam.setPipeline(calibrator);
        webcam.open();


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                saveCalibration(calibrator.getAnalysis());
                telemetry.log().add("Saved calibration data.");
            }
        }
    }

    private static String serialize(Scalar scalar) {
        StringBuilder builder = new StringBuilder();
        for(Double i : scalar.val) {
            builder.append(",").append(i.toString());
        }
        return builder.toString();
    }

    private static Scalar deserialize(String string) {
        android.util.Log.i("Calibration", "Deserializing calibration data:");
        android.util.Log.i("Calibration", string);
        String[] split = string.split(",");
        return new Scalar(
                Double.parseDouble(split[1]),
                Double.parseDouble(split[2]),
                Double.parseDouble(split[3])
        );
    }

    public static void saveCalibration(Scalar color) {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        ReadWriteFile.writeFile(file, serialize(color));
    }

    public static Scalar getCalibration() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        return deserialize(ReadWriteFile.readFile(file));
    }
}
