package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm Component. Currently uses hard-coded calibration data.
 */
public class Arm {
    private final DcMotor arm;
    private final Servo wrist;
    private final CRServo collector;

    /**
     * Constructs a new arm from the main lift motor, the wrist servo, and the collector servo.
     * @param arm main lift motor
     * @param wrist wrist servo
     * @param collector collector servo (continuous)
     */
    public Arm(DcMotor arm, Servo wrist, CRServo collector) {
        this.arm = arm;
        this.wrist = wrist;
        this.collector = collector;
    }

    /**
     * Levels that the arm can reach. The Cap level is used for capping, whereas the
     * Top, Middle, and Bottom levels are the 3 levels on the hub. The Ground level is used for
     * picking up objects.
     *
     * @see #toLevel(HubLevel, boolean)
     */
    public enum HubLevel {
        Cap,
        Top,
        Middle,
        Bottom,
        Ground,
    }

    /**
     * Collector modes. The Eject and Collect modes are self-explanatory, i.e. they
     * eject and collect blocks in the collector. The Stop state is used to stop the collector from
     * spinning.
     *
     * @see #setCollectorMode(CollectorMode)
     */
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
                collector.setPower(-0.5);
            case Collect:
                collector.setPower(0.5);
            case Stop:
                collector.setPower(0);
        }
    }

    /**
     * Sets the raw positions of the arm and wrist. Note that these positions might not be achieved immediately.
     * This is a private helper function, for internal use only.
     * @param arm position of the arm motor
     * @param wrist position of the wrist servo
     */
    private void setArmPosition(int arm, double wrist) {
        this.arm.setTargetPosition(arm); // Set the target position we are going to run to
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Tell the motor we want it to run to a position
        this.wrist.setPosition(wrist); // Get the wrist in the correct place
        this.arm.setPower(1); // Set the power and begin running to that position
    }

    /**
     * Begins moving towards the specified level.
     * Note that this function is asynchronous and will complete before the arm achieves
     * the desired position.
     * @param level level to move towards
     * @param front front load or back load (<code>false</code> is back load)
     */
    public void toLevel(@NonNull HubLevel level, boolean front) {
        switch (level) {
            case Cap:
                // TODO: Implement Cap level
            case Top:
                if (front) {
                    // TODO: Get calibration for levels from config, add calibration OpMode?
                    setArmPosition(-2200, 0.53);

                } else {
                    setArmPosition(-4100, 0.36);
                }
            case Middle:
                if (front) {
                    // TODO: Implement front load
                } else {
                    setArmPosition(-5238, 0.44);
                }
            case Bottom:
                if (front) {
                    // TODO: Implement front load
                } else {
                    setArmPosition(-6300, 0.49);
                }
            case Ground:
                setArmPosition(0, 0.53);
        }
    }

    /**
     * @param level level to move towards
     * @see #toLevel(HubLevel, boolean)
     */
    public void toLevel(@NonNull HubLevel level) {
        toLevel(level, false);
    }
}
