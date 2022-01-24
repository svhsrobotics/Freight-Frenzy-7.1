package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm Component. Currently uses hard-coded calibration data.
 */
public class Arm {
    public final DcMotor arm;
    public final Servo wrist;
    public final CRServo collector;

    private static final int MAX_ARM = 1000;
    public static final int ARM_OFFSET = 0;

    public Arm(DcMotor arm, Servo wrist, CRServo collector) {
        this.arm = arm;
        this.wrist = wrist;
        this.collector = collector;
    }

    public enum HubPosition {
        // TODO: Implement capping
        FRONTTOP,
        FRONTMID,
        FRONTBOT,
        FRONTLOAD,

        BACKTOP,
        BACKMID,
        BACKBOT,
        BACKLOAD,

        START
    }

    // Reset the encoder when the arm is in the initial position
    public void reset() {
        this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Go to the position, check done for status
    public void goToPosition(HubPosition position) {
        switch (position) {
            case START:
                setPositions(0, -1.0);

            case FRONTLOAD:
                setPositions(0, -0.8);
        }
    }

    // Are we done moving to the position yet? (Only arm, not wrist)
    public boolean done() {
        return this.arm.getCurrentPosition() == this.arm.getTargetPosition(); // should we allow for some difference??
    }

    // Helper for goToPosition
    public void setPositions(int arm, double wrist) {
        setPosition(this.arm, arm + ARM_OFFSET, MAX_ARM);
        this.wrist.setPosition(wrist);
    }

    public void setArmPosition(int arm) {
        setPosition(this.arm, arm, MAX_ARM);
    }

    public void setWristPosition(double wrist) {
        this.wrist.setPosition(wrist);
    }

    // DcMotor as Servo helpers

    private void setPosition(DcMotor motor, int position, int max) {
        setPosition(motor, position, 1.0, max);
    }

    private void setPosition(DcMotor motor, int position, double power, int max) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.getCurrentPosition();
    }




    // COLLECTOR

    public enum CollectorMode {
        Eject,
        Collect,
        Stop,
    }

    /**
     * Sets the collector mode.
     * @param mode collector mode to activate
     */
    public void setCollectorMode(@NonNull CollectorMode mode) {
        switch (mode) {
            case Eject:
                // TODO: Figure out which one of these should be negative
                collector.setPower(-0.15);
                break;
            case Collect:
                collector.setPower(0.15);
                break;
            case Stop:
                collector.setPower(0);
                break;
        }
    }
}
