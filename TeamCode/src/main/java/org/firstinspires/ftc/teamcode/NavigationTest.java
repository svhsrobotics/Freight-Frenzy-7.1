/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="NavigationTest")

public class NavigationTest extends LinearOpMode {
    LinearOpMode opMode;

   // public HardwareMap hardwareMap; // will be set in OpModeManager.runActiveOpMode
private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        // Initialize the hardware
        robot.initDrives();
        robot.initIMU();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot, this);

        waitForStart();

        long nanos = System.nanoTime();
        // Move to the left until we encounter the carousel
        drive.navigationMonitorExternal(0.125, 8, 8, 0, 1000, () -> {
            // Don't run until a second has passed
            if (System.nanoTime() - nanos > 1000000000) {
                // If the acceleration is greater than 1.0, stop moving
                return robot.imu.getLinearAcceleration().xAccel > 1.0;
            }
            return false;
        });

        //drive.setTargetAngle(0);

        //Vuforia vuforia = new Vuforia(this);

        // Send telemetry message to signify robot waiting;

//        drive.stopResetEncoder();
//        drive.runUsingEncoder();

/*
        double cycleMillisNow = System.currentTimeMillis();
        double cycleMillisPrior = cycleMillisNow;

        drive.navigationMonitorTicks(1, 0, 32, 30);

        cycleMillisNow = System.currentTimeMillis();
        double cycleMillisDelta = cycleMillisNow - cycleMillisPrior;
        final double DISTANCE= 32;
        double speed = DISTANCE/(cycleMillisDelta/1000);
        Log.i("Speed", "......................................................");
        Log.i("Speed", "Time Elapsed:" +(cycleMillisDelta/1000));
        Log.i("Speed", "Speed = "+ speed +" inches/second");
*/

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);
        //Drive2 drive = new Drive2(this);
        //drive.init();
/*
        //final Drive3 drive = new Drive3(this);
        Robot robot = new Robot(hardwareMap);
        // Initialize the hardware
        robot.initDrives();
        robot.initIMU();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot, this);
        //drive.setTargetAngle(0);

        //Vuforia vuforia = new Vuforia(this);

        // Send telemetry message to signify robot waiting;

//        drive.stopResetEncoder();
//        drive.runUsingEncoder();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        double cycleMillisNow = System.currentTimeMillis();
        double cycleMillisPrior = cycleMillisNow;

        drive.navigationMonitorTicks(30, 0, 30, 30);

         cycleMillisNow = System.currentTimeMillis();
        double cycleMillisDelta = cycleMillisNow - cycleMillisPrior;
        final double DISTANCE= 50;
        double speed = DISTANCE/(cycleMillisDelta/1000);
        Log.i("Speed", "......................................................");
        Log.i("Speed", "Time Elapsed:" +(cycleMillisDelta/1000));
        Log.i("Speed", "Speed = "+ speed +" inches/second");


        /* while(opModeIsActive()){
            drive.getImuAngle();
            sleep(500);
        }
        if(1+1 == 2) return; */
/*
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
   /*     if (opModeIsActive()) {

            //drive.setNewTargetPosition(48, 48);
            //drive.turnOnRunToPosition();

            //drive.setNewTargetPosition(32, 32);
            //drive.vroom_vroom(.75, Math.PI/2, .75, Math.PI/2, 10.0);


            Thread rotate = new Thread(new Runnable() {
                @Override
                public void run() {
                    Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
                    drive.navigationMonitorTicks(0, 10, 10, 30);
                }
            });

            rotate.start();
            sleep(1000);
            //for(int i = 1; i < 121; i++){
            for(int i = 1; i < 31; i++){
                sleep(150);
                Log.i("DriveByEncoderOpMode", i+"");
                drive.setTargetAngle(i*3);
            }
            for(int i = 30; i >= 0; i--){
                sleep(150);
                Log.i("DriveByEncoderOpMode", i+"");
                drive.setTargetAngle(i*3);
            }
            for(int i = 1; i < 31; i++){
                sleep(150);
                Log.i("DriveByEncoderOpMode", i+"");
                drive.setTargetAngle(i*3);
            }
            for(int i = 30; i >= 0; i--){
                sleep(150);
                Log.i("DriveByEncoderOpMode", i+"");
                drive.setTargetAngle(i*3);
            }
*/

            //*/

//            drive.ceaseMotion();
//            sleep(2000);

//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(.5, -24, -24, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(.5, 0, 24, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(.5, 24, -24, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(.5, 0, 24, 10);

//            drive.ceaseMotion();
//            sleep(2000);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, 0, -24, 10);
//            drive.ceaseMotion();
//            sleep(2000);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, 48, 0, 10);
//            drive.ceaseMotion();
//            sleep(2000);
//            drive.vroomVroomMonitorTicks(1, -48, 0, 10);

            //Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            //vuforia.shootPowerShots();

//            drive.vroomVroomMonitorTicks(1, 12, 12, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, -12, 12, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, 0, 33, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(2/3, 0, 1, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1/3, 0, 1, 10);
            drive.ceaseMotion();
            sleep(1000);
            if(1+1 == 2) return;
            //each block individually from starting position
            //block 1
            //drive.vroomVroomMonitorTicks(1, 3, 3, 10);
            //Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            //drive.vroomVroomMonitorTicks(1, 0, 68, 10);

            //block 2
            //drive.vroomVroomMonitorTicks(1, 3, 3, 10);
            //Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            //drive.vroomVroomMonitorTicks(1, 0, 68, 10);
            //Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            //drive.vroomVroomMonitorTicks(1, -24, 24, 10);

            //block 3
            //drive.vroomVroomMonitorTicks(1, 3, 3, 10);
            //Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            //drive.vroomVroomMonitorTicks(1, 0, 116, 10);


            //all together now
//            drive.vroomVroomMonitorTicks(1, 3, 3, 10);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, 0, 68, 10);
//            drive.ceaseMotion();
//            sleep(1000);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, -24, 22, 10);
//            drive.ceaseMotion();
//            sleep(1000);
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            drive.vroomVroomMonitorTicks(1, 24, 24, 10);
//
//
//
//            drive.ceaseMotion();
//            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
//            //sleep(2000);
//
//            drive.ceaseMotion();
        }

//        drive.turnOffRunToPosition();
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //sleep(40000);     // pause for servos to move

        //telemetry.addData("Path", "Complete");
        //telemetry.update();
    }
