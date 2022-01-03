package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final DcMotor arm;
    private final Servo wrist;
    private final CRServo collector;

    public Arm(DcMotor arm, Servo wrist, CRServo collector) {
        this.arm = arm;
        this.wrist = wrist;
        this.collector = collector;
    }

    public enum HubLevel {
        Cap,
        Top,
        Middle,
        Bottom,
        Ground,
    }

    public enum CollectorMode {
        Eject,
        Collect,
        Stop,
    }

    public void setCollector(CollectorMode mode) {
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

    private void setArmPosition(int arm, double wrist) {
        this.arm.setTargetPosition(arm); // Set the target position we are going to run to
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Tell the motor we want it to run to a position
        this.wrist.setPosition(wrist); // Get the wrist in the correct place
        this.arm.setPower(1); // Set the power and begin running to that position
    }

    public void goToLevel(HubLevel level, boolean front) {
        switch (level) {
            case Cap:
                // TODO: Implement Cap level
            case Top:
                if (front) {
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
}
