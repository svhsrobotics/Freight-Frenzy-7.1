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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Scan Servo LM", group = "Concept")
@Disabled
public class ConceptScanServoLM extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    double MAX_V =  1.0;     // Maximum rotational position
    double MIN_V =  -1.0;     // Minimum rotational position

    // Define class members
    CRServo rightCarousel;
    CRServo leftCarousel;
    double velocity = (MAX_V - MIN_V) / 2; // Start at halfway position
    boolean rampUp = true;




    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel"); // CHS1
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel"); // CHS0
        //gamepad = hardwareMap.get(Gamepad.class, "gamepad1");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        velocity = 0;
        long time = System.currentTimeMillis();
        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.

            /*if (rampUp = true) {
                // Keep stepping up until we hit the max value.
                velocity += INCREMENT ;
                if (velocity >= MAX_V) {
                    velocity = MAX_V;
                    rampUp = false;   // Switch ramp direction
                }
            }
            else if (rampUp = false){
                // Keep stepping down until we hit the min value.
                velocity -= INCREMENT ;
                if (velocity <= MIN_V) {
                    velocity = MIN_V;
                    rampUp = true;  // Switch ramp direction
                }
            }*/
            if (gamepad1.right_bumper) {
                velocity = MAX_V;
            }
            if (gamepad1.left_bumper) {
                velocity = MIN_V;
            }
            if (gamepad1.dpad_up && MAX_V < 1 && System.currentTimeMillis() - time >= 150) {
                MAX_V = MAX_V + .05;
                MIN_V = MIN_V - .05;
                if( velocity == MAX_V -.05) {
                    velocity = MAX_V;
                }else {
                    velocity = MIN_V;
                    }

                time = System.currentTimeMillis();
            }
            if (gamepad1.dpad_down && MAX_V > 0 && System.currentTimeMillis() - time >= 150) {
                MAX_V = MAX_V - .05;
                MIN_V = MIN_V + .05;
                if( velocity == MAX_V +.05) {
                    velocity = MAX_V;
                }else {
                        velocity = MIN_V;
                }
                time = System.currentTimeMillis();
            }
            if(gamepad1.b) {
                velocity = 0;
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", velocity);
            telemetry.addData("Max clockwise", "%5.2f", MAX_V);
            telemetry.addData("Max counterclockwise", "%5.2f", MIN_V);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            rightCarousel.setPower(velocity);
            leftCarousel.setPower(velocity);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
