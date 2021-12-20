package org.firstinspires.ftc.teamcode.op1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceCalibration extends LinearOpMode {
    @Override
    public void runOpMode(){
        DistanceSensor distanceLeftFront = hardwareMap.get(DistanceSensor.class,"DistanceLF");
        DistanceSensor distanceLeftSide = hardwareMap.get(DistanceSensor.class,"DistanceLS");
        DistanceSensor distanceRightFront = hardwareMap.get(DistanceSensor.class,"DistanceRF");
        DistanceSensor distanceRightSide = hardwareMap.get(DistanceSensor.class,"DisatcmeRS");

        double DistanceLF = 0;
        double DistanceLS =0;
        double DistanceRF = 0;
        double DistanceRS =0;

        waitForStart();
        boolean currentA = false;
        boolean currentB = false;
        while(opModeIsActive()){
            currentA = gamepad1.a;
            currentB = gamepad1.b;
            if (currentA==true){
                //create an array with the data of the right front sensor
              double  arrayRF[] = new double[5];
              for(int i=0;i<5;i++){
                  DistanceRF =  distanceRightFront.getDistance(DistanceUnit.INCH);
                  arrayRF[i]= DistanceRF;
                  sleep(10);

              }
                //craete an array with the data of the left side sensor
                double  arrayLS[] = new double[5];
                for(int i=0;i<5;i++){
                    DistanceLS =  distanceLeftSide.getDistance(DistanceUnit.INCH);
                    arrayLS[i]= DistanceLS;
                    sleep(10);

                }
            }
            if (currentB==true){
                //create an array with the data of the left front sensor
                double  arrayLF[] = new double[5];
                for(int i=0;i<5;i++){
                    DistanceLF =  distanceLeftFront.getDistance(DistanceUnit.INCH);
                    arrayLF[i]= DistanceLF;
                    sleep(10);

                }
                //craete an array with the data of the left side sensor
                double  arrayRS[] = new double[5];
                for(int i=0;i<5;i++){
                    DistanceRS =  distanceRightSide.getDistance(DistanceUnit.INCH);
                    arrayRS[i]= DistanceRS;
                    sleep(10);

                }
            }
        }
    }
}
