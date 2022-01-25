package org.firstinspires.ftc.teamcode.navigation;

public class DriveSet<T> {
    public T rightFront;
    public T rightBack;
    public T leftFront;
    public T leftBack;

    public DriveSet(T rightFront, T rightBack, T leftFront, T leftBack) {
        set(rightFront, rightBack, leftFront, leftBack);
    }

    public void set(T rightFront, T rightBack, T leftFront, T leftBack) {
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.leftFront = leftFront;
        this.leftBack = leftBack;
    }
}
