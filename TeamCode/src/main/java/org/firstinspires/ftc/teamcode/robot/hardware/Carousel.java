package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.CRServo;

public class Carousel {
    CRServo motor;

    public double trim;

    public Carousel(CRServo motor, double trim) {
        this.motor = motor;
        this.trim = trim;
    }

    public Carousel(CRServo motor) {
        this.motor = motor;
        this.trim = 0.0;
    }

    public void stop() {
        this.motor.setPower(0);
    }

    public void clockwise() {
        this.motor.setPower(1.0 + this.trim);
    }

    public void cclockwise() {
        this.motor.setPower(-1.0 + this.trim);
    }
}
