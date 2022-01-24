package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Handles the final approach to the carousel.
 * You should be reasonably sure that you are in front of the carousel.
 */
public class CarouselLocalizer {
    static final double TARGET_ACCEL = 1.0; // Minimum acceleration at which we assume we hit the carousel

    private final Robot robot;
    private final Drive2 drive;

    public CarouselLocalizer(Robot robot, Drive2 drive) {
        this.robot = robot;
        this.drive = drive;
    }

    /**
     * Move forward until we get to the carousel. Uses acceleration in order to tell when we get there.
     * @param timeout time in ms until we abort
     */
    public void localize(double timeout) {
        drive.navigationMonitorExternal(0.25, 100, 0, timeout, () -> {
            double accel = robot.imu.getLinearAcceleration().xAccel;
            if (accel > TARGET_ACCEL) {
                android.util.Log.d("CarouselLocalizer", "Final acceleration: " + accel);
                return true;
            } else {
                return false;
            }
        });

        drive.ceaseMotion();
    }
}
