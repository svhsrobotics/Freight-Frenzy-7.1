package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Wrapper around DcMotor providing enhanced and commonly used functionality such as resetting the encoder and calculating the total number of inches traveled.
 */
public class Drive {
    private final DcMotor motor;
    private final double inchCounts;

    /**
     * Constructs a new Drive with specified motor, wheel, and gear reduction.
     * @param motor the DcMotor object to wrap
     * @param revCounts the number of counts in one motor revolution
     * @param gearReduction the reduction ratio of the drive's gearing
     * @param wheelDiameter the diameter of the wheel, in inches
     */
    public Drive(DcMotor motor, double revCounts, double gearReduction, double wheelDiameter) {
        this.motor = motor;
        this.inchCounts = (revCounts * gearReduction) / (wheelDiameter * Math.PI);
    }

    /**
     * Sets the drive's power level
     * @param power power level to set the motor to
     */
    public void setPower(double power) {
        this.motor.setPower(power);
    }

    /**
     * Sets the drives mode
     * @param mode mode to set the motor to
     */
    public void setMode(DcMotor.RunMode mode) {
        this.motor.setMode(mode);
    }

    /**
     * Sets the direction of the motor
     * @param direction
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    /**
     * Gets the current drive mode
     * @return current mode
     */
    public DcMotor.RunMode getMode() {
        return this.motor.getMode();
    }

    /**
     * Gets the current reading the of drive's encoder
     * @return current encode reading
     */
    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    /**
     * Gets the number of inches traveled since the last encoder reset.
     * Calculated by dividing the result of <code>getCurrentPosition()</code> by <code>inchCounts</code>.
     * @return inches traveled
     */
    public double getCurrentInches() {
        return this.getCurrentPosition() / this.inchCounts;
    }

    /**
     * Resets the encoder to zero. This will remove power to the motor temporarily, use with caution. It attempts to resume it's previous mode after reset.
     */
    public void resetEncoder() {
        DcMotor.RunMode current = this.getMode();
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMode(current);
    }
}
