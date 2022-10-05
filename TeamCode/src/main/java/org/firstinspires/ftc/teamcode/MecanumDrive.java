package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/****************************
    runOpMode(): Code inside this method will run exactly once after you press the INIT button. This is where you should put all code for the OpMode.
    waitForStart(): This method pauses the Op-Mode until you press the START button on the driver station.
    isStarted(): returns true if the START button has been pressed, otherwise it returns false.
    isStopRequested(): returns true if the STOP button has been pressed, otherwise it returns false.
    idle(): calls Thread.yield, allowing other threads at the same priority level to run.
    opModeIsActive(): returns isStarted() && !isStopRequested() and calls idle().
    opModeInInit(): returns !isStarted() && !isStopRequested() and does not call idle().
 *****************************/

@TeleOp(name = "Mecanum Drive", group="Linear OpMode")
public class MecanumDrive extends LinearOpMode {

    // private ElapsedTime runtime = new ElapsedTime(); //Added from BasicOpLinear
    Servo grabby;

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); //Added from BasicOpLinear
        telemetry.update(); //Added from BasicOpLinear

        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(0.5);


        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        // Both right side motors should be going in one direction, and both left side motors going in the opposite direction
        /* This appears to be set already in the hardware map
        */
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


    waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.y) {
                grabby.setPosition(0.5);
            }
            if (gamepad1.a){
                grabby.setPosition(0);
            }

            telemetry.update();

            /* This may not be optimal. Consider using
            // Uses the left thumbstick for forward & backwards robot movement
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */


            double vertical = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
            double horizontal = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
            double pivot = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement

            // Variables for wheel motor power and inputs
            /*
            double frontLeftPower = (vertical + horizontal + pivot);
            double backLeftPower = (vertical - horizontal + pivot);
            double frontRightPower = (vertical - horizontal - pivot);
            double backRightPower = (vertical + horizontal - pivot);
            */
            double frontLeftPower = (vertical + horizontal + pivot);
            double backLeftPower = (vertical - horizontal - pivot);
            double frontRightPower = (vertical - horizontal - pivot);
            double backRightPower = (vertical + horizontal + pivot);

            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            // leftDrive.setPower(leftPower);
            // rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            /* Taking out for simplicity
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); //Added from BasicOpLinear
            telemetry.update();
             */

            /*
            // lift pseudocode
            int rotations = how many rotations it takes to go up a notch;
            if (VALUE.UP && currentPos < maxPos){ // if the controller says 'go up' the lift goes up one mode as long as that is possible
            currentPos = currentPos + rotations;
            telemetry.addData("lift", currentPos);
            }

            if (VALUE.DOWN && currentPos > minPos){ // if the controller says 'go up' the lift goes up one mode as long as that is possible
            currentPos = currentPos - rotations;
            telemetry.addData("lift", currentPos);
                        */

        }
    }
} // End of Class