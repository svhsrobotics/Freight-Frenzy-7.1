package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;

@TeleOp(name= "Distance Calibartion", group= "Teleop")
public class DistanceCalibration extends LinearOpMode {
    @Override
    public void runOpMode(){
        DistanceSensor distanceLeftFront = hardwareMap.get(DistanceSensor.class,"DistanceLF");
        DistanceSensor distanceLeftSide = hardwareMap.get(DistanceSensor.class,"DistanceLS");
        DistanceSensor distanceRightFront = hardwareMap.get(DistanceSensor.class,"DistanceRF");
        DistanceSensor distanceRightSide = hardwareMap.get(DistanceSensor.class,"DistanceRS");

        double DistanceLF = 0;
        double DistanceLS =0;
        double DistanceRF = 0;
        double DistanceRS =0;

        Configuration config = Configurator.load();

        waitForStart();
        boolean currentA = false;
        boolean currentB = false;
        double totalRF = 0;
        double totalLS = 0;
        double totalLF = 0;
        double totalRS = 0;
        while(opModeIsActive()){
            currentA = gamepad1.a;
            currentB = gamepad1.b;
            telemetry.addLine("Press A for Blue Carousel");
            telemetry.addLine("Press B for Red Carousel");
            if (currentA==true){
                //create an array with the data of the right front sensor
              double  arrayRF[] = new double[5];
              for(int i=0;i< arrayRF.length;i++){
                  DistanceRF =  distanceRightFront.getDistance(DistanceUnit.INCH);
                  arrayRF[i]= DistanceRF;
                  totalRF = totalRF + arrayRF[i];
                  sleep(10);

              }
                //craete an array with the data of the left side sensor
                double  arrayLS[] = new double[5];
                for(int i=0;i< arrayLS.length;i++){
                    DistanceLS =  distanceLeftSide.getDistance(DistanceUnit.INCH);
                    arrayLS[i]= DistanceLS;
                    totalLS =totalLS + arrayLS[i];
                    sleep(10);

                }
                double averageRF= totalLF/ arrayRF.length;
                double averageLS = totalRS/ arrayLS.length;
                telemetry.addData("Right Front Distance:", averageRF);
                telemetry.addData("Left Side Distance:", averageLS);
                telemetry.update();
                config.rfdistance = averageRF;
                config.lsdistance = averageLS;
                Configurator.save(config);
                currentA=false;
            }
            if (currentB==true){
                //create an array with the data of the left front sensor
                double  arrayLF[] = new double[5];
                for(int i=0;i< arrayLF.length;i++){
                    DistanceLF =  distanceLeftFront.getDistance(DistanceUnit.INCH);
                    arrayLF[i]= DistanceLF;
                    totalLF = totalLF + arrayLF[i];
                    sleep(10);

                }
                //craete an array with the data of the left side sensor
                double  arrayRS[] = new double[5];
                for(int i=0;i< arrayRS.length;i++){
                    DistanceRS =  distanceRightSide.getDistance(DistanceUnit.INCH);
                    arrayRS[i]= DistanceRS;
                    totalRS = totalRS + arrayRS[i];
                    sleep(10);

                }
                double averageLF= totalLF/ arrayLF.length;
                double averageRS = totalRS/ arrayRS.length;
                telemetry.addData("Left Front Distance:", averageLF);
                telemetry.addData("Right Side Distance:", averageRS);
                telemetry.update();
                config.lfdistance = averageLF;
                config.rsdistance = averageRS;
                Configurator.save(config);
                currentB=false;

                //put averages in the json file
            }
            if (config.lfdistance != null) {
                telemetry.addData("left front saved as", config.lfdistance);
            }else if (config.lsdistance != null) {
                telemetry.addData("left side saved as", config.lsdistance);
            }else if(config.rfdistance != null) {
                telemetry.addData("right front saved as", config.rfdistance);
            }else if(config.rsdistance != null) {
                telemetry.addData("right side saved as", config.rsdistance);
            }
            telemetry.update();
        }
    }
}
