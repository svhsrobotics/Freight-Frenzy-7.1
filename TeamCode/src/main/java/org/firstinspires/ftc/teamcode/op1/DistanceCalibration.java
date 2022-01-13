package org.firstinspires.ftc.teamcode.op1;

import android.nfc.Tag;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
@Disabled
@TeleOp(name= "Distance Calibration", group= "Teleop")
public class DistanceCalibration extends LinearOpMode {
    @Override
    public void runOpMode(){
        DistanceSensor distanceLeftFront = hardwareMap.get(DistanceSensor.class,"distance left");
   //     DistanceSensor distanceLeftSide = hardwareMap.get(DistanceSensor.class,"distance right");
  //     DistanceSensor distanceRightFront = hardwareMap.get(DistanceSensor.class,"DistanceRF");
        DistanceSensor distanceRightSide = hardwareMap.get(DistanceSensor.class,"distance right");

        double DistanceLF = 0;
  //      double DistanceLS =0;
  //      double DistanceRF = 0;
        double DistanceRS =0;



        waitForStart();


        Configuration config = Configurator.load();

        if (config == null) {
            config = new Configuration();
        }
        if(config.rfdistance == null){
            config.rfdistance = new Double(0.0);
        }
        if(config.lfdistance == null){
            config.lfdistance = new Double(0.0);
        }
        if(config.rsdistance == null){
            config.rsdistance = new Double(0.0);
        }
        if(config.lsdistance == null) {
            config.lsdistance = new Double(0.0);
        }
        Configurator.save(config);

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
    /*        if (currentA==true){
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
                config.rfdistance = new Double(averageRF);
                config.lsdistance = new Double(averageLS);
                Configurator.save(config);
                currentA=false;
                break:
            }
     */       if (currentB==true){
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
                config.lfdistance = new Double(averageLF);
                config.rsdistance = new Double(averageRS);
                Configurator.save(config);
                currentB=false;

                //put averages in the json file
                break;
            }
            if (config.lfdistance != null) {
                Log.i ("Double","left front saved as: "+ config.lfdistance);
             //   break;
            }
            if (config.lsdistance != null) {
                Log.i ("Double","left side saved as: "+ config.lsdistance);
             //   break;
            }
 //           if(config.rfdistance != null) {
 //               Log.i ("Double","right front saved as: "+ config.rfdistance);
 //               break;
            }
            if(config.rsdistance != null) {
                Log.i ("Double","right side saved as: "+ config.rsdistance);
               // break;
            }
            telemetry.update();
        }
    }

