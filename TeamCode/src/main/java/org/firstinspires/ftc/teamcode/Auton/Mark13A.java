/**
 * Auton using Vuforia camera code and a simple claw response.
 * No objects detected should open claw and stay open
 * Signal one should open & close the claw once, Two should do it twice, Three.. you guessed it
 */

package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Constants.lift_floor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 */
@Autonomous(name = "Mark 13A", group = "Linear OpMode")
// @Disabled
public class Mark13A extends LinearOpMode {

    private Servo grabby;
    private DcMotor lift;
    private ElapsedTime runtime = new ElapsedTime();

    //Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub
    BNO055IMU imu;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */

    // Moved to Constants already so should be ok to take out at some point
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    // TODO
    int parkPosition;

    /** Moved to constants
     * private static final String VUFORIA_KEY = "";
     */

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

    /**
     * Added by Scoy
     * Using booleans to hopefully make it easier to do the default parking
     * if all three are signals are still false after the detection period
     * then the robot does a default park (e.g. A1 or A3)
     */
    private boolean signalOne = false;
    private boolean signalTwo = false;
    private boolean signalThree = false;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
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
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        // telemetry.addData(">", "Press Play to start op mode");
        // telemetry.update();

        telemetry.addData("Status", "Camera Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(OPEN);
        lift.setTargetPosition(lift_floor);

        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * Both right side motors should be going in one direction,
         * and both left side motors going in the opposite direction
         */

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            /*
                             * SCOY added
                             * Each signal is set to default of false up above,
                             * but this should toggle the true/false values.
                             * These are potentially unneeded but it made more sense to me
                             * when testing the logic farther down in the code.
                             **/
                            if (recognition.getLabel().equals("1 Bolt")) {
                                signalOne = true;
                            } else if (recognition.getLabel().equals("2 Bulb")) {
                                signalTwo = true;
                            } else if (recognition.getLabel().equals("3 Panel")) {
                                signalThree = true;
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }

                        telemetry.update();

                    }

                }


                /*
                 * If you JUST want to test the logic
                 * without the camera dynamically testing for signals,
                 *  Remove the comments one at a time for each signal variable
                 * */
                // signalOne = true;
                // signalTwo = true;
                // signalThree = true;


                // Conditional sequence with basic logic and actions
                if (signalOne == false && signalTwo == false && signalThree == false) {
                    grabby.setPosition(CLOSED);

                    /*
                     * Testing purposes. This would be replaced by a specific parking function.
                     * This should open the claw, pause, and close one time
                     */
                }   else if (signalOne == true) {
                    grabby.setPosition(CLOSED);
                    sleep(500);
                    grabby.setPosition(OPEN);

                    /*
                     * Testing purposes. This would be replaced by a specific parking function.
                     * This should open the claw, pause, and close twice
                     */
                }   else if (signalTwo == true) {
                    grabby.setPosition(CLOSED);
                    sleep(500);
                    grabby.setPosition(OPEN);
                    sleep(500);
                    grabby.setPosition(CLOSED);
                    sleep(500);
                    grabby.setPosition(OPEN);

                    /*
                     * Testing purposes. This would be replaced by a specific parking function.
                     * This should open the claw, pause, and close three times
                     */
                }   else if (signalThree == true) {
                    grabby.setPosition(OPEN);
                    sleep(500);
                    grabby.setPosition(CLOSED);
                    sleep(500);
                    grabby.setPosition(OPEN);
                    sleep(500);
                    grabby.setPosition(CLOSED);
                    sleep(500);
                    grabby.setPosition(OPEN);
                    sleep(500);
                    grabby.setPosition(CLOSED);

                }

            } // END of testing sequence




        } // end of opModeIsActive
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    /*
     * Functions to run for autonomous parking
     * These functions should be used to replace specific code in the logic testing
     * which should make it make it more reusable in various autonomous files
     * without messing with the logic */
    private void signalOnePark() {
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
    }

    private void signalTwoPark() {
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
        sleep(500);
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
    }

    private void signalThreePark() {
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
        sleep(500);
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
        sleep(500);
        grabby.setPosition(OPEN);
        sleep(500);
        grabby.setPosition(CLOSED);
    }

} // end of class

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
