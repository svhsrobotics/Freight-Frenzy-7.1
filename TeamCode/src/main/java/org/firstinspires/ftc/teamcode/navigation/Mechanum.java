package org.firstinspires.ftc.teamcode.navigation;

import static java.lang.Math.PI;

public class Mechanum {

    /**
     * Calculates the power factor from a theta value.
     * For "right" facing mech. wheels, i.e. front right and back left.
     */
    public static double rightPowerFactor(double theta) {
        if (theta > 0 && theta < PI/2) {
            return -Math.cos(2 * theta);
        } else if (theta >= -PI && theta < -PI/2) {
            return Math.cos(2 * theta);
        } else if (theta >= PI/2 && theta <= PI) {
            return 1;
        } else {
            return -1;
        }
    }

    /**
     * Calculates the power factor from a theta value.
     * For "left" facing mech. wheels, i.e. front left and back right.
     */
    public static double leftPowerFactor(double theta) {
        if (theta > -PI/2 && theta < 0) {
            return Math.cos(2 * theta);
        } else if(theta > PI/2 && theta < PI) {
            return -Math.cos(2 * theta);
        } else if(theta >= 0 && theta <= PI/2) {
            return 1;
        } else {
            return -1;
        }
    }
}
