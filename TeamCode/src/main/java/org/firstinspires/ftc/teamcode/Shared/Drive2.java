package org.firstinspires.ftc.teamcode.Shared;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.robot.testRobot;

import java.util.HashMap;

public class Drive2 {
    String TAG = "Drive";
    public final Drive leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    final BNO055IMU imu;

    LinearOpMode opMode;
    Telemetry telemetry;
    public HardwareMap hardwareMap; // will be set in OpModeManager.runActiveOpMode
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP //For test robot
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP //For competition robot
    static final double     WHEEL_DIAMETER_INCHES   = 6 ;     // For figuring circumference  //For test robot
    //static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference //For competition robot
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;

    HashMap <Drive, Integer> motorInitialPositions, motorTargetPositions;
    HashMap <Drive, Double> motorPowerFactors;

    static double imuSecondOpModeAdjustment = 0;
    Orientation lastAngles = new Orientation();

    Robot robot;

    double TotalMotorCurrent;


    public Drive2(Robot robot, LinearOpMode opMode){
        this.robot = robot;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.imu = robot.imu;

        this.leftFrontDrive = robot.Drives.get(Robot.DrivePos.FRONT_LEFT);
        this.rightFrontDrive = robot.Drives.get(Robot.DrivePos.FRONT_RIGHT);
        this.leftBackDrive = robot.Drives.get(Robot.DrivePos.BACK_LEFT);
        this.rightBackDrive = robot.Drives.get(Robot.DrivePos.BACK_RIGHT);

        motorPowerFactors = new HashMap<>();
        setTargetAngle(0);
    }

    public void check_and_set_drive(double magRight, double thetaRight, double magLeft, double thetaLeft) {
        double rightFrontPowerFactor, leftFrontPowerFactor, rightBackPowerFactor, leftBackPowerFactor;
        double pi = Math.PI;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path",  "Starting at %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // reset the timeout time and start motion.
        runtime.reset();

        if(thetaRight > 0 && thetaRight < pi/2){
            rightFrontPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= -pi && thetaRight < -pi/2){
            rightFrontPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight >= pi/2 && thetaRight <= pi){
            rightFrontPowerFactor = 1;
        }else{
            rightFrontPowerFactor = -1;
        }

        if(thetaLeft > 0 && thetaLeft < pi/2) {
            leftBackPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= -pi && thetaLeft < -pi/2){
            leftBackPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= pi/2 && thetaLeft <= pi){
            leftBackPowerFactor = 1;
        }else{
            leftBackPowerFactor = -1;
        }

        if(thetaRight > -pi/2 && thetaRight < 0) {
            rightBackPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight > pi/2 && thetaRight < pi){
            rightBackPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= 0 && thetaRight <= pi/2){
            rightBackPowerFactor = 1;
        }else{
            rightBackPowerFactor = -1;
        }

        if(thetaLeft > -pi/2 && thetaLeft < 0) {
            leftFrontPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft > pi/2 && thetaLeft < pi){
            leftFrontPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= 0 && thetaLeft <= pi/2){
            leftFrontPowerFactor = 1;
        }else{
            leftFrontPowerFactor = -1;
        }

        motorPowerFactors.put(leftFrontDrive, leftFrontPowerFactor);
        motorPowerFactors.put(leftBackDrive, leftBackPowerFactor);
        motorPowerFactors.put(rightFrontDrive, rightFrontPowerFactor);
        motorPowerFactors.put(rightBackDrive, rightBackPowerFactor);

        //Speeds speeds = getPhiSpeeds((magLeft + magRight)/2);
        setMotorPowers(magLeft, magRight);

    }

    /**navigationByPhi
     *
     * @param targetSpeed Desired Speed
     * @param targetTheta Desired Orientation of Robot
     *
     * Utilizes orientation away from the desired location(phi) in order to alter power power facotrs on the motors.
     */
    public void navigationByPhi(double targetSpeed, double targetTheta) {
        double rightFrontPowerFactor, leftFrontPowerFactor, rightBackPowerFactor, leftBackPowerFactor;
        double pi = Math.PI;
        double thetaRight = targetTheta;
        double thetaLeft = targetTheta;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path",  "Starting at %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // reset the timeout time and start motion.
        runtime.reset();

        if(thetaRight > 0 && thetaRight < pi/2){
            rightFrontPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= -pi && thetaRight < -pi/2){
            rightFrontPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight >= pi/2 && thetaRight <= pi){
            rightFrontPowerFactor = 1;
        }else{
            rightFrontPowerFactor = -1;
        }

        if(thetaLeft > 0 && thetaLeft < pi/2) {
            leftBackPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= -pi && thetaLeft < -pi/2){
            leftBackPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= pi/2 && thetaLeft <= pi){
            leftBackPowerFactor = 1;
        }else{
            leftBackPowerFactor = -1;
        }

        if(thetaRight > -pi/2 && thetaRight < 0) {
            rightBackPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight > pi/2 && thetaRight < pi){
            rightBackPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= 0 && thetaRight <= pi/2){
            rightBackPowerFactor = 1;
        }else{
            rightBackPowerFactor = -1;
        }

        if(thetaLeft > -pi/2 && thetaLeft < 0) {
            leftFrontPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft > pi/2 && thetaLeft < pi){
            leftFrontPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= 0 && thetaLeft <= pi/2){
            leftFrontPowerFactor = 1;
        }else{
            leftFrontPowerFactor = -1;
        }

        motorPowerFactors.put(leftFrontDrive, leftFrontPowerFactor);
        motorPowerFactors.put(leftBackDrive, leftBackPowerFactor);
        motorPowerFactors.put(rightFrontDrive, rightFrontPowerFactor);
        motorPowerFactors.put(rightBackDrive, rightBackPowerFactor);

        SpeedsPhi speedsPhi = getPhiSpeeds(targetSpeed, motorPowerFactors);
        setMotorPowersPhi(speedsPhi);
    }


    //               ^
    //               |  Y axis
    //               |
    //    ---------------------
    //    |        Front      |
    //    |                   |
    //    |                   |   X axis
    //    |        Robot      |----------->
    //    |                   |
    //    |      top view     |
    //    ---------------------

    /** navigationMonitorTicks
     *
     * @param speed
     * @param xInches
     * @param yInches
     * @param timeout
     *
     * Utilizes wheel encoders in order to track the location of the robot.
     * Imu heading tracks the direction the robot is pointing.
     * Tracking the location with the wheel encoders allows for a direct input for location and time in order to direect the robot's movement.
     */
    public void navigationMonitorTicks(double speed, double xInches, double yInches, double timeout) {
        //Borrowed Holonomic robot navigation ideas from https://www.bridgefusion.com/blog/2019/4/10/robot-localization-dead-reckoning-in-f  irst-tech-challenge-ftc
        //    Robot Localization -- Dead Reckoning in First Tech Challenge (FTC)
        double theta = Math.atan2(yInches, xInches);
        double magnitude = Math.hypot(xInches, yInches);
        int tickCountPriorLeftFront = leftFrontDrive.getCurrentPosition(), tickCountPriorLeftBack = leftBackDrive.getCurrentPosition();
        int tickCountPriorRightFront = rightFrontDrive.getCurrentPosition(), tickCountPriorRightBack = rightBackDrive.getCurrentPosition();
        int ticksTraveledLeftFront = 0, ticksTraveledLeftBack = 0, ticksTraveledRightFront = 0, ticksTraveledRightBack = 0;
        double inchesTraveledX = 0, inchesTraveledY = 0, inchesTraveledTotal = 0, rotationInchesTotal = 0;
        double cycleMillisNow = 0, cycleMillisPrior = System.currentTimeMillis(), cycleMillisDelta, startMillis = System.currentTimeMillis();
        //vroom_vroom(speed, theta, speed, theta);
        getImuAngle();
        navigationByPhi(speed, theta);
        adjustThetaInit();
        //setTargetAngle(mImuCalibrationAngle);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && inchesTraveledTotal <= magnitude){
//For Speed Changing
            double initialTime= opMode.getRuntime();

            TotalMotorCurrent = leftFrontDrive.getCurrent();
            TotalMotorCurrent += leftBackDrive.getCurrent();
            TotalMotorCurrent += rightFrontDrive.getCurrent();
            TotalMotorCurrent += rightBackDrive.getCurrent();
            Log.i(TAG,"navigationMonitorTicks: Total Motor Current= "+ TotalMotorCurrent);


            int tickCountNowLeftFront = leftFrontDrive.getCurrentPosition();
            int tickCountNowLeftBack = leftBackDrive.getCurrentPosition();
            int tickCountNowRightFront = rightFrontDrive.getCurrentPosition();
            int tickCountNowRightBack = rightBackDrive.getCurrentPosition();
            int deltaTicksLeftFront = tickCountNowLeftFront - tickCountPriorLeftFront;
            int deltaTicksLeftBack = tickCountNowLeftBack - tickCountPriorLeftBack;
            int deltaTicksRightFront = tickCountNowRightFront - tickCountPriorRightFront;
            int deltaTicksRightBack = tickCountNowRightBack - tickCountPriorRightBack;
            ticksTraveledLeftFront += deltaTicksLeftFront;
            ticksTraveledLeftBack += deltaTicksLeftBack;
            ticksTraveledRightFront += deltaTicksRightFront;
            ticksTraveledRightBack += deltaTicksRightBack;
            double leftFrontInchesDelta = deltaTicksLeftFront / COUNTS_PER_INCH;
            double rightFrontInchesDelta = -deltaTicksRightFront / COUNTS_PER_INCH;  //Minus sign converts to holonomic drive perspective
            double rightBackInchesDelta = -deltaTicksRightBack / COUNTS_PER_INCH;  //Minus sign converts to holonomic drive perspective
            double leftBackInchesDelta = deltaTicksLeftBack / COUNTS_PER_INCH;
            double rotationAvgInchesDelta = (leftFrontInchesDelta + rightFrontInchesDelta + rightBackInchesDelta + leftBackInchesDelta)/4;
            rotationInchesTotal += rotationAvgInchesDelta;
            double leftFrontRobotInchesDelta = leftFrontInchesDelta - rotationAvgInchesDelta;
            double rightFrontRobotInchesDelta = rightFrontInchesDelta - rotationAvgInchesDelta;
            double rightBackRobotInchesDelta = rightBackInchesDelta - rotationAvgInchesDelta;
            double leftBackRobotInchesDelta = leftBackInchesDelta - rotationAvgInchesDelta;
            double deltaInchesRobotX = (leftFrontRobotInchesDelta + rightFrontRobotInchesDelta - rightBackRobotInchesDelta - leftBackRobotInchesDelta) / (2 * Math.sqrt(2));
            double deltaInchesRobotY = (leftFrontRobotInchesDelta - rightFrontRobotInchesDelta - rightBackRobotInchesDelta + leftBackRobotInchesDelta) / (2 * Math.sqrt(2));
            double deltaInchesRobot = Math.hypot(deltaInchesRobotX, deltaInchesRobotY);
            double FUDGE_FACTOR = 36/51.0;
            inchesTraveledX += deltaInchesRobotX * FUDGE_FACTOR;
            inchesTraveledY += deltaInchesRobotY * FUDGE_FACTOR;
            inchesTraveledTotal += Math.hypot(deltaInchesRobotX * FUDGE_FACTOR, deltaInchesRobotY * FUDGE_FACTOR);

/*
Working on Changing Speed to be consistent
 */
            double timePassed = opMode.getRuntime()-initialTime;//Length of time of the loop
            double actualSpeedX = deltaInchesRobotX/timePassed;//gets the actual speed in inches/second in x direction
            double actualSpeedY = deltaInchesRobotY/timePassed;//gets the actual speed in inches/second in y direction
            double speedScale=24;//Speed in inches/second at speed=1
            double speedScaled= speed*speedScale;//Speed in terms of inches/second instead of 0 to 1

            double actualSpeed = Math.sqrt((actualSpeedX*actualSpeedX)+(actualSpeedY*actualSpeedY));//Take hypotenuse of speed in x and y
            Log.i("Speed", String.format("Actual Speed in/s:, %.2f", actualSpeed));//log actual speed
            double speedFactor= (actualSpeed-speedScaled)/speedScaled;//percent error of speed
            speedScaled = speedScaled*(1+speedFactor);//corrects adjusted speed wiht percent error
            Log.i("Speed", String.format("New Speed in/s:, %.2f", speedScaled));//log actual speed
            speed=speedScaled/speedScale;//converts adjusted speed to 0 to 1 scale
            Log.i("Speed", String.format("New Speed from 0 to 1:, %.2f", speed));//log actual speed

            //Provide feedback to keep robot moving in right direction based on encoder ticks
            adjustTheta(xInches, yInches, speed, inchesTraveledX, inchesTraveledY);  //Must call adjustThetaInit() before a loop with adjustTheta()


            cycleMillisNow = System.currentTimeMillis();
            cycleMillisDelta = cycleMillisNow - cycleMillisPrior;
            cycleMillisPrior = cycleMillisNow;
            telemetry.addData("Ticks Traveled (lf, lb)", "%7d, %7d", ticksTraveledLeftFront, ticksTraveledLeftBack);
            telemetry.addData("Ticks Traveled (rf, rb)", "%7d, %7d", ticksTraveledRightFront, ticksTraveledRightBack);
            telemetry.addData("In Traveled (X, Y)", "X: %.1f, Y: %.1f", inchesTraveledX, inchesTraveledY);
            telemetry.addData("In Traveled (Tot, Rot)", "%.1f, %.1f", inchesTraveledTotal,rotationInchesTotal);
            telemetry.addData("Cycle Millis:", "%4f", cycleMillisDelta);
            telemetry.update();
//            Log.i("Drive", String.format("Ticks Traveled (lf, lb): %7d, %7d", ticksTraveledLeftFront, ticksTraveledLeftBack));
//            Log.i("Drive", String.format("Ticks Traveled (rf, rb): %7d, %7d", ticksTraveledRightFront, ticksTraveledRightBack));
//            Log.i("Drive", String.format("Ticks Delta (lf, lb): %7d, %7d", deltaTicksLeftFront, deltaTicksLeftBack));
//            Log.i("Drive", String.format("Ticks Delta (rf, rb): %7d, %7d", deltaTicksRightFront, deltaTicksRightBack));
//            Log.i("Drive", String.format("In Traveled (X, Y): X: %.2f, Y: %.2f", inchesTraveledX, inchesTraveledY));
//            Log.i("Drive", String.format("In Traveled (Tot, Rot): %.2f, %.2f", inchesTraveledTotal,rotationInchesTotal));
//            Log.i("Drive", String.format("Incremental Speed (in/sec): %.2f", deltaInchesRobot/cycleMillisDelta * 1000));
//            Log.i("Drive", String.format("Cycle Millis: %.3f, Total Seconds: %.3f", cycleMillisDelta, (System.currentTimeMillis() - startMillis)/1000));



            tickCountPriorLeftFront = tickCountNowLeftFront;
            tickCountPriorLeftBack = tickCountNowLeftBack;
            tickCountPriorRightFront = tickCountNowRightFront;
            tickCountPriorRightBack = tickCountNowRightBack;
        }
    }

    /**stopResetEncoder
     * Performs "STOP_AND_RESET_ENCODER" on all drive motors
     */
    public void stopResetEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**runUsingEncoder
     * Sets all drive motors to RUN_USING_ENCODER
     * Sets zero power behavior to float, allowing for no active force resisting rotation
     */
    public void runUsingEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**ceaseMotion
     *Stop all motion;
     */
    public void ceaseMotion(){
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    /**setMotorPowers
     *Set power factors of all drive motors using magLeft and magRight
     */
    public void setMotorPowers(double magLeft, double magRight){
        leftFrontDrive.setPower(motorPowerFactors.get(leftFrontDrive) * magLeft);
        rightFrontDrive.setPower(motorPowerFactors.get(leftBackDrive) * magRight);
        leftBackDrive.setPower(motorPowerFactors.get(rightFrontDrive) * magLeft);
        rightBackDrive.setPower(motorPowerFactors.get(rightBackDrive) * magRight);
    }
    /**setMotorPowersPhi
     *Set power factors of all drive motors using speedsPhi
     */
    public void setMotorPowersPhi(SpeedsPhi speedsPhi){
        leftFrontDrive.setPower(speedsPhi.leftFrontSpeed);
        rightFrontDrive.setPower(speedsPhi.rightFrontSpeed);
        leftBackDrive.setPower(speedsPhi.leftBackSpeed);
        rightBackDrive.setPower(speedsPhi.rightBackSpeed);
    }

    private double thetaErrorSum;
    public void adjustThetaInit() { thetaErrorSum = 0; }
    public void adjustTheta(double targetX, double targetY, double targetSpeed, double nowX, double nowY){
        if(nowX == 0 && nowY == 0 && mTargetAngle == 0){
            Log.i("Drive", "adjustTheta: nowX and nowY are both still zero so not computing an adjustment factor yet");
            return;
        }
        //double GAIN = 0.6, THETA_ERROR_SUM_MAX = Math.PI/4/GAIN; //Max error sum is +/- 45 degrees
        double targetTheta = Math.atan2(targetY, targetX);
        double nowTheta = Math.atan2(nowY, nowX);
//        if((targetTheta - nowTheta > 0 && thetaErrorSum < THETA_ERROR_SUM_MAX) || (targetTheta - nowTheta < 0 && thetaErrorSum > -THETA_ERROR_SUM_MAX))
//            thetaErrorSum += Math.min(Math.max(targetTheta - nowTheta, -THETA_ERROR_SUM_MAX), THETA_ERROR_SUM_MAX);  //Max allowed increment is max value
//        double adjustedTargetTheta = targetTheta + GAIN * thetaErrorSum;

        //The cosine function acts as the gain so that as the error angle approaches 180 degrees, the error factor goes to zero,
        // since the motors will already be pulling in the opposite direction.
        double angleDifference = calculateAngleDifference(targetTheta, nowTheta);
        double adjustedTargetTheta = getEulerAngle(targetTheta + Math.cos(angleDifference/2) * angleDifference);
        //Speeds speeds = getSpeeds(targetSpeed, nowTheta);
//        vroom_vroom(targetSpeed, adjustedTargetTheta, targetSpeed, adjustedTargetTheta);
        //vroom_vroom(speeds.rightSpeed, adjustedTargetTheta, speeds.leftSpeed, adjustedTargetTheta);
        navigationByPhi(targetSpeed, adjustedTargetTheta);
//        vroom_vroom(targetSpeed, targetTheta, targetSpeed, targetTheta);
        Log.i("Drive", String.format("IMU angle: %.2f, adj angle: %.2f", mCurrentImuAngle, mAdjustedAngle));
        Log.i("Drive", String.format("adjustTheta: (degrees) target: %.4f, now: %.4f, adjusted: %.4f, error: %.4f",
                targetTheta/Math.PI*180, nowTheta/Math.PI*180, adjustedTargetTheta/Math.PI*180, thetaErrorSum/Math.PI*180));
    }

    double mCurrentImuAngle, mPriorImuAngle, mTargetAngle, mAdjustedAngle, mPriorAdjustedAngle, mImuCalibrationAngle;

    public void setTargetAngle(double targetAngle){
        mPriorImuAngle = mTargetAngle = targetAngle + imuSecondOpModeAdjustment;
    }

    /**
     *We experimentally determined the Z axis is the axis we want to use for heading angle.
     *We have to process the angle because the imu works in euler angles so the Z axis is
     *returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
     *180 degrees. We detect this transition and track the total cumulative angle of rotation.
     * @return 0
     */
    public double getImuAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double adjustedHeading = angles.firstAngle + imuSecondOpModeAdjustment;
        telemetry.addData("IMU Angles (X/Y/Z)", "%.1f / %.1f / %.1f", angles.secondAngle, angles.thirdAngle, angles.firstAngle);
        telemetry.update();
        Log.i(TAG, String.format("getImuAngle: IMU Angles (X/Y/Z): %.1f / %.1f / %.1f", angles.secondAngle, angles.thirdAngle, angles.firstAngle));
        return mCurrentImuAngle = angles.firstAngle;

    }

    /**
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getImuDeltaAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double deltaAngle = (mCurrentImuAngle = getImuAngle()) - mPriorImuAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        return deltaAngle;
    }

    /**
     *
     * @return New adjusted angle used to create adjusted power levels
     */
    public double getAdjustedAngle(){
        return mAdjustedAngle = getEulerAngleDegrees(getImuAngle());
    }

    /**
     * Convert an angle such that -pi <= angle <= pi
     */
    private double getEulerAngle(double angle){
        if(angle < -Math.PI) return angle%(2*Math.PI) + 2 * Math.PI;
        else if (angle > Math.PI) return angle%(2*Math.PI) - 2 * Math.PI;
        else return angle;
    }

    /**
     * Convert an angle such that -pi <= angle <= pi
     */
    private double getEulerAngleDegrees(double angle){
        if(angle < -180) return angle%360 + 360;
        else if (angle > 180) return angle%360 - 180;
        else return angle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double getPowerCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double angleError, powerCorrection, angle, gain;

        angle = getAdjustedAngle();  //IMU angle converted to Euler angle (IMU may already deliver Euler angles)

        angleError = mTargetAngle - angle;        // reverse sign of angle for correction.

        gain = Math.max(-0.05*Math.abs(angleError) + 0.1, .05);  //Varies from .2 around zero to .05 for errors above 10 degrees
        powerCorrection = angleError * gain;

        return powerCorrection;
    }

    /**
     * See if we are moving in a straight line and if not return an angle correction value phiCorrection.
     * @return Phi adjustment, + is rotate counter clockwise; - is rotate clockwise.
     */
    private double getPhiCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double GAIN_HIGH_RANGE = 0.6, GAIN_LOW_RANGE = 2.0;
        double angleError, phiCorrection, angle, gain;

        angle = getAdjustedAngle();

        angleError = mTargetAngle - angle;        // reverse sign of angle for correction.

        gain = Math.max(-(GAIN_LOW_RANGE - GAIN_HIGH_RANGE)*Math.abs(angleError) + GAIN_LOW_RANGE,
                GAIN_HIGH_RANGE);  //Varies from 2.0 around zero to .6 for errors above 10 degrees
        phiCorrection = angleError * gain;

        return phiCorrection;
    }

    private enum Side{
        Left, Right
    }

    private class Speeds{
        public double rightSpeed;
        public double leftSpeed;
        public Speeds(double right, double left){
            rightSpeed = right;
            leftSpeed = left;
        }
    }

    private class SpeedsPhi{
        public double rightFrontSpeed;
        public double leftFrontSpeed;
        public double rightBackSpeed;
        public double leftBackSpeed;
        public SpeedsPhi(double leftFront, double leftBack, double rightFront, double rightBack){
            leftFrontSpeed = leftFront;
            leftBackSpeed = leftBack;
            rightFrontSpeed = rightFront;
            rightBackSpeed = rightBack;
        }
    }

    /**
     * Get the adjusted speeds for each side of the robot to allow it to turn enough to stay on a straight line. Only call once per cycle.
     * @param targetSpeed
     * @return
     */
    private Speeds getSpeeds(double targetSpeed, double nowAngle){
        int sign = nowAngle > 0 ? 1 : -1;
        double powerCorrection = getPowerCorrection() * sign;
        //double powerCorrection = 0;
        double adjustedLeftSpeed, adjustedRightSpeed;
        if(targetSpeed + powerCorrection > 1){
            adjustedRightSpeed = 1;
            adjustedLeftSpeed = 1 - 2 * powerCorrection; // power - (2 * powerCorrection - (1 - power))
        } else if (targetSpeed - powerCorrection > 1){
            adjustedRightSpeed = 1 + 2 * powerCorrection;
            adjustedLeftSpeed = 1;
        } else {
            adjustedRightSpeed = targetSpeed + powerCorrection;
            adjustedLeftSpeed = targetSpeed - powerCorrection;
        }
        mPriorImuAngle = mCurrentImuAngle;
        mPriorAdjustedAngle = mAdjustedAngle;
        Log.i(TAG, String.format("getSpeeds: targetSpeed: %.3f, powerCorrection: %.3f", targetSpeed, powerCorrection));
        Log.i(TAG, String.format("getSpeeds: adjustedLeftSpeed: %.3f, adjustedRightSpeed: %.3f", adjustedLeftSpeed, adjustedRightSpeed));
        return new Speeds(adjustedRightSpeed, adjustedLeftSpeed);
    }

    /**
     * Get the adjusted speeds for each side of the robot to allow it to turn enough to stay on a straight line. Only call once per cycle.
     * @param targetSpeed
     * @return
     */
    private SpeedsPhi getPhiSpeeds(double targetSpeed, HashMap<Drive, Double> motorPowerFactors){
        double powerCorrection = getPowerCorrection();
        double adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed;

        if(motorPowerFactors.get(leftFrontDrive)*targetSpeed - powerCorrection > 1){
            adjustedLeftFrontSpeed = 1;
        }else if(motorPowerFactors.get(leftFrontDrive)*targetSpeed - powerCorrection < -1) {
            adjustedLeftFrontSpeed = -1;
        }else{
            adjustedLeftFrontSpeed = motorPowerFactors.get(leftFrontDrive) * targetSpeed - powerCorrection;
        }

        if(motorPowerFactors.get(leftBackDrive)*targetSpeed - powerCorrection > 1){
            adjustedLeftBackSpeed = 1;
        }else if(motorPowerFactors.get(leftBackDrive)*targetSpeed - powerCorrection < -1) {
            adjustedLeftBackSpeed = -1;
        }else{
            adjustedLeftBackSpeed = motorPowerFactors.get(leftBackDrive) * targetSpeed - powerCorrection;
        }

        if(motorPowerFactors.get(rightFrontDrive)*targetSpeed + powerCorrection > 1){
            adjustedRightFrontSpeed = 1;
        }else if(motorPowerFactors.get(rightFrontDrive)*targetSpeed + powerCorrection < -1) {
            adjustedRightFrontSpeed = -1;
        }else{
            adjustedRightFrontSpeed = motorPowerFactors.get(rightFrontDrive) * targetSpeed + powerCorrection;
        }

        if(motorPowerFactors.get(rightBackDrive)*targetSpeed + powerCorrection > 1){
            adjustedRightBackSpeed = 1;
        }else if(motorPowerFactors.get(rightBackDrive)*targetSpeed + powerCorrection < -1) {
            adjustedRightBackSpeed = -1;
        }else{
            adjustedRightBackSpeed = motorPowerFactors.get(rightBackDrive) * targetSpeed + powerCorrection;
        }

        mPriorImuAngle = mCurrentImuAngle;
        mPriorAdjustedAngle = mAdjustedAngle;
        Log.i(TAG, String.format("getSpeedsPhi: targetSpeed: %.3f, powerCorrection: %.3f", targetSpeed, powerCorrection));
        Log.i(TAG, String.format("getSpeedsPhi: adjustedLeftFrontSpeed: %.3f, adjustedLeftBackSpeed: %.3f, adjustedRightFrontSpeed: %.3f, adjustedRightBackSpeed: %.3f",
                adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed));
        Log.i(TAG, String.format("getSpeedsPhi: leftFrontMotorPowerFactor: %.3f, leftBackMotorPowerFactor: %.3f, rightFrontMotorPowerFactor: %.3f, rightBackMotorPowerFactor: %.3f",
                motorPowerFactors.get(leftFrontDrive), motorPowerFactors.get(leftBackDrive), motorPowerFactors.get(rightFrontDrive), motorPowerFactors.get(rightBackDrive)));
        return new SpeedsPhi(adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed);
    }


    /**
     *
     * @param targetAngle
     * @param nowAngle
     * @return Difference in target angle and nowAngle
     */
    private double calculateAngleDifference(double targetAngle, double nowAngle){
        double angle1 = 0, angle2 = 0, angleDiff180 = 0, angleDiff0 = 0, angleDifference = 0, returnAngle = 0;
        if(targetAngle >= 0 && nowAngle <= 0){
            angle1 = Math.PI - targetAngle;
            angle2 = nowAngle + Math.PI;
            angleDiff180 = angle1 + angle2;
            angleDiff0 = targetAngle - nowAngle;
            angleDifference = Math.min(angleDiff180, angleDiff0);
            if(angleDiff180 < angleDiff0)
                angleDifference *= -1;
        }else if(targetAngle <= 0 && nowAngle >= 0){
            angle1 = Math.PI - nowAngle;
            angle2 = targetAngle + Math.PI;
            angleDiff180 = angle1 + angle2;
            angleDiff0 = nowAngle - targetAngle;
            angleDifference = Math.min(angleDiff180, angleDiff0);
            if(angleDiff0 < angleDiff180)
                angleDifference *= -1;
        }else{
            // for all cases that target & now have the same sign
            angleDifference = targetAngle - nowAngle;
        }
        Log.i(TAG, String.format("calculateAngleDifference: angleDiff0: %.2f, angleDiff180: %.2f, angleDifference: %.2f",
                angleDiff0, angleDiff180, angleDifference));
        return angleDifference;
    }
}