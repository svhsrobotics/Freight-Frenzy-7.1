package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.EnumMap;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.HardwareNotFoundException;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Timeout;

public class DettmarRobot extends Robot{

    // Hardware definitions
    /*
    static final private class Hardware {
        static final double revCounts = 28;
        static final double gearReduction = 40;//40 before
        static final double wheelDiameter = 6;//3 for test robot

        static final String frontLeftMotorName = "left_front_drive"; // CHM2
        static final String frontRightMotorName = "right_front_drive"; // CHM3
        static final String backLeftMotorName = "left_back_drive"; // CHM0
        static final String backRightMotorName = "right_back_drive"; // CHM1
        static final String imuName = "imu";
    }*/

    public DettmarRobot(HardwareMap hardwareMap, Logger logger) {
        super(hardwareMap, logger);
    }

    public DettmarRobot(HardwareMap hardwareMap) {
        super(hardwareMap);
        /*
        Hardware.frontLeftMotorName = "left_front_drive"; // CHM2
        Hardware.frontRightMotorName = "right_front_drive"; // CHM3
        Hardware.backLeftMotorName = "left_back_drive"; // CHM0
        Hardware.backRightMotorName = "right_back_drive"; // CHM1

         */
    }
}
