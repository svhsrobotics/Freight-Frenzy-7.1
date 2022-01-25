package org.firstinspires.ftc.teamcode.navigation;

public class Vector {
    public final double magnitude;
    public final double theta;

    public Vector(double x, double y) {
        this.theta = Math.atan2(y, x);
        this.magnitude = Math.hypot(x, y);
    }
}
