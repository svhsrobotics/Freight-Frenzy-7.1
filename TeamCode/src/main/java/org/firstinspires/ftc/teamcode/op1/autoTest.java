package org.firstinspires.ftc.teamcode.op1;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Shared.Drive2;

import java.util.List;

@Autonomous
public class autoTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUmiib//////AAABmbrFAeBqbkd/hmTwBoU6jXFUjeYK8xAgeYu6r9ZuLmpgb4tqNl4oIhXkpbBXmCusnhPlxJ3DHEkExTnQKhvCU49Yu2jslI6vaQ+V5F21ZAbbBod6lm9zyBEpkujo7IOq2TdOaJSIdN5wW3zxrHTksfrzBuKZKRsArompruh7jrm/B4W3F/EunA8ymkVoi29W84q81XMwJyonWlS2sd3pebXvLW0YOKmA63QgdmtSpp9XVAccwiH8ND8rk7FXlIIucim1Ig5FmVPLIx88t7doptXh8uiXfHHMqXc1T1MrRvfemYaUqyg7I5lYLNjLhuRmBZO3BM/qoyjPhpMVtGNh6+z3VgaKhP7O6zI07W0mmMfO";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        Drive2 drive = new Drive2(this);
        drive.init();
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        double SPEED = 1;
        waitForStart();
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    drive.navigationMonitorTicks(SPEED / 2, 0, 4, 5);
                    Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
                    drive.navigationMonitorTicks(SPEED, 60, 0, 30);
                    Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");

                }else{
                    drive.navigationMonitorTicks(SPEED / 2, 0, 4, 5);
                    Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
                    drive.navigationMonitorTicks(SPEED / 2, 0, -4, 5);
                    Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
                }
            }
        }
    }



    private void initVuforia() {
                /*
                 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
                 */
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = VUFORIA_KEY;
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

                //  Instantiate the Vuforia engine
                vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Loading trackables is not necessary for the TensorFlow Object Detection engine.
            }

            /**
             * Initialize the TensorFlow Object Detection engine.
             */
            private void initTfod() {
                int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfodParameters.minResultConfidence = 0.8f;
                tfodParameters.isModelTensorFlow2 = true;
                tfodParameters.inputSize = 320;
                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
            }
    }

