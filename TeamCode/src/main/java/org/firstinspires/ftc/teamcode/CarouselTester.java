package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Robot.DrivePos;
import org.firstinspires.ftc.teamcode.util.Timeout;
@TeleOp
public class CarouselTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //DcMotorEx carousel = hardwareMap.get(DcMotorEx.class, "rightCarousel");

        //Robot robot = new Robot(hardwareMap);
        //robot.initHardware();
        //Drive2 drive = new Drive2(robot, this);
        //drive.navigationByPhi(0.125, 0);
        // Get the imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        LynxDcMotorController motorController = hardwareMap.getAll(LynxDcMotorController.class).get(0);

        waitForStart();
        motorController.setMotorPower(0, 0.25);
        motorController.setMotorPower(1, -0.25);
        motorController.setMotorPower(2, 0.25);
        motorController.setMotorPower(3, -0.25);

        while (opModeIsActive()) {
            telemetry.addData("IMU X-Accel", imu.getLinearAcceleration().xAccel);
            telemetry.addData("IMU Y-Accel", imu.getLinearAcceleration().yAccel);
            telemetry.addData("Motor Current", motorController.getMotorCurrent(0, CurrentUnit.AMPS));
            android.util.Log.i("ACCEL", String.valueOf(imu.getLinearAcceleration().xAccel));
            //android.util.Log.i("YACCEL", String.valueOf(imu.getLinearAcceleration().yAccel));
            //android.util.Log.i("CURRENT", String.valueOf(motorController.getMotorCurrent(0, CurrentUnit.AMPS)));
            if (imu.getLinearAcceleration().xAccel > 1) {
                break;
            }
            sleep(50);
        }
    }
}
