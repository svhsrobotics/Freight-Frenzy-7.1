package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.util.EnumMap;

public class testRobot {

    // Hardware definitions
    static final private class Hardware {
        static final double revCounts = 28;
        static final double gearReduction = 40;
        static final double wheelDiameter = 3;

        static final String frontLeftMotorName = "left_front_drive";
        static final String frontRightMotorName = "right_front_drive";
        static final String backLeftMotorName = "left_back_drive";
        static final String backRightMotorName = "right_back_drive";
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

    public testRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initHardware() {
        initDrives();
        initIMU();
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

    public void initDrives() {
        this.Drives.put(DrivePos.FRONT_LEFT,
                new Drive(this.hardwareMap.get(DcMotor.class, Hardware.frontLeftMotorName),
                        Hardware.revCounts, Hardware.gearReduction, Hardware.wheelDiameter));
        this.Drives.put(DrivePos.FRONT_RIGHT,
                new Drive(this.hardwareMap.get(DcMotor.class, Hardware.frontRightMotorName),
                        Hardware.revCounts, Hardware.gearReduction, Hardware.wheelDiameter));
        this.Drives.put(DrivePos.BACK_LEFT,
                new Drive(this.hardwareMap.get(DcMotor.class, Hardware.backLeftMotorName),
                        Hardware.revCounts, Hardware.gearReduction, Hardware.wheelDiameter));
        this.Drives.put(DrivePos.BACK_RIGHT,
                new Drive(this.hardwareMap.get(DcMotor.class, Hardware.backRightMotorName),
                        Hardware.revCounts, Hardware.gearReduction, Hardware.wheelDiameter));

        for (Drive drive : this.Drives.values())
            drive.run();

        this.Drives.get(DrivePos.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        this.Drives.get(DrivePos.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
