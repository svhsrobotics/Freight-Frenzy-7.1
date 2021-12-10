package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

    static final String FILENAME = "calibrateTargetColor.txt";

    @Override
    public void runOpMode() {
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        TeamElementCalibrator calibrator = new TeamElementCalibrator();
        webcam.setPipeline(calibrator);
        webcam.open();

        //Gamepad gamepad = hardwareMap.get(Gamepad.class, "gamepad1");

        waitForStart();

        /*while (opModeIsActive()) {
            if (gamepad.a) {
                saveCalibration(calibrator.getAnalysis());
            }
        }*/
        HashMap<TeamElementDetector.TeamElementPosition, Region> regions = new HashMap<TeamElementDetector.TeamElementPosition, Region>();
        regions.put(TeamElementDetector.TeamElementPosition.LEFT, new Region(new Point(320 / 3 * 0.0, 0.0), 320 / 3.0, 240.0));
        Calibration cal = new Calibration(regions, new HSVColor(1.0,2.0,3.0), 4.0);
        save(cal.serialize());
    }

    private static String serialize(Scalar scalar) {
        StringBuilder builder = new StringBuilder();
        for(Double i : scalar.val) {
            builder.append(",").append(i.toString());
        }
        return builder.toString();
    }

    private static Scalar deserialize(String string) {
        String[] split = string.split(",");
        return new Scalar(
                Double.parseDouble(split[0]),
                Double.parseDouble(split[1]),
                Double.parseDouble(split[2])
        );
    }

    public static void saveCalibration(Scalar color) {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        ReadWriteFile.writeFile(file, serialize(color));
    }

    public static void save(String data) {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        ReadWriteFile.writeFile(file, data);
    }

    public static Scalar getCalibration() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        return deserialize(ReadWriteFile.readFile(file));
    }
}
