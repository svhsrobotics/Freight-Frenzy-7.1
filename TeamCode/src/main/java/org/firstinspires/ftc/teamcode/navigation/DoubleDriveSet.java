package org.firstinspires.ftc.teamcode.navigation;

public class DoubleDriveSet extends DriveSet<Double> {

    public DoubleDriveSet(Double rightFront, Double rightBack, Double leftFront, Double leftBack) {
        super(rightFront, rightBack, leftFront, leftBack);
    }

    public void add(DriveSet<Double> other) {
        set(rightFront + other.rightFront, rightBack + other.rightBack, leftFront + other.leftFront, leftBack + other.leftBack);
    }

    public void addAll(Double other) {
        set(rightFront + other, rightBack + other, leftFront + other, leftBack + other);
    }
}
