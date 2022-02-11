package org.firstinspires.ftc.teamcode.util;

public class ExMath {
    public static double square_with_sign(double input) {
        if (input > 0) {
            return input * input;
        } else {
            return input * input * -1;
        }
    }
}
