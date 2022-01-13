package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous
public class FindHardware extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxDcMotorController> motorControllerList = hardwareMap.getAll(LynxDcMotorController.class);
        for (LynxDcMotorController controller : motorControllerList) {
            telemetry.log().add((String) hardwareMap.getNamesOf(controller).toArray()[0]);
            for (int i = 0; i < 4; i++) {
                telemetry.log().add("Motor: " + i);
                controller.setMotorPower(i, 100);
                sleep(1000);
                controller.setMotorPower(i, 0);
                sleep(2000);
            }
        }
        List<LynxServoController> servoControllerList = hardwareMap.getAll(LynxServoController.class);
        for (LynxServoController controller : servoControllerList) {
            telemetry.log().add((String) hardwareMap.getNamesOf(controller).toArray()[0]);
            for (int i = 0; i < 4; i++) {
                telemetry.log().add("Servo: " + i);
                controller.setServoPosition(i, 0);
                sleep(1000);
                controller.setServoPosition(i, 1);
                sleep(2000);
            }
        }
    }
}
