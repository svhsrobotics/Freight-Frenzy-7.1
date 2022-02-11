package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.HardwareNotFoundException;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.util.EnumMap;

public class Robot {

    // Hardware definitions
    static final private class Hardware {
        static final double revCounts = 28;
        static final double gearReduction = 40;//40 before
        static final double wheelDiameter = 6;//3 for test robot

        //Competition Robot
//        static final String frontLeftMotorName = "FL"; // CHM2
//        static final String frontRightMotorName = "FR"; // CHM3
//        static final String backLeftMotorName = "BL"; // CHM0
//        static final String backRightMotorName = "BR"; // CHM1
//        static final String imuName = "imu";

        static final String frontLeftMotorName = "FL"; // CHM2 //left_front_drive
        static final String frontRightMotorName = "FR"; // CHM3 //right_front_drive
        static final String backLeftMotorName = "BL"; // CHM0 //left_back_drive
        static final String backRightMotorName = "BR"; // CHM1 //right_back_drive
        static final String imuName = "imu";
    }

    private HardwareMap hardwareMap;

    public enum DrivePos {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public EnumMap<DrivePos, Drive> Drives = new EnumMap<DrivePos, Drive>(DrivePos.class);

    public BNO055IMU imu;

    public Arm arm;

    private final Logger logger;

    public Robot(HardwareMap hardwareMap, Logger logger) {
        this.hardwareMap = hardwareMap;
        this.logger = logger;
    }

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.logger = new Logger();
    }

    public void initHardware() throws IllegalArgumentException {
        try {
            initDrives();
            initArm();
            initIMU();
        } catch (IllegalArgumentException e) {
            throw new HardwareNotFoundException(e);
        }
    }

    public void initIMU() {
        // Get the imu
        this.imu = hardwareMap.get(BNO055IMU.class, Hardware.imuName);

        // Set the IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Initialize the imu with specified parameter
        imu.initialize(parameters);

        // Wait while the gyro is calibrating...
        while (!imu.isGyroCalibrated()) {
            Thread.yield();
        }

        Timeout t = new Timeout(5);
        while (!imu.isAccelerometerCalibrated() && !t.timeout()) {
            Thread.yield();
        }
    }

    public void initArm() {
        this.arm = new Arm(
                this.hardwareMap.get(DcMotor.class, "Arm"), // EXM0
                this.hardwareMap.get(Servo.class, "pivotCollector"), // CHS3
                this.hardwareMap.get(CRServo.class, "spinCollector") // CHS2
        );
    }


    public void initDrives() {
        this.Drives.put(DrivePos.FRONT_LEFT,
                new Drive(this.hardwareMap.get(DcMotorEx.class, Hardware.frontLeftMotorName),
                        Hardware.revCounts,Hardware.gearReduction,Hardware.wheelDiameter));
        this.Drives.put(DrivePos.FRONT_RIGHT,
                new Drive(this.hardwareMap.get(DcMotorEx.class, Hardware.frontRightMotorName),
                        Hardware.revCounts,Hardware.gearReduction,Hardware.wheelDiameter));
        this.Drives.put(DrivePos.BACK_LEFT,
                new Drive(this.hardwareMap.get(DcMotorEx.class, Hardware.backLeftMotorName),
                        Hardware.revCounts,Hardware.gearReduction,Hardware.wheelDiameter));
        this.Drives.put(DrivePos.BACK_RIGHT,
                new Drive(this.hardwareMap.get(DcMotorEx.class, Hardware.backRightMotorName),
                        Hardware.revCounts,Hardware.gearReduction,Hardware.wheelDiameter));

        for (Drive drive : this.Drives.values())
            drive.run();

        this.Drives.get(DrivePos.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        this.Drives.get(DrivePos.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
