package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/****************************
 runOpMode():
 Code inside this method will run exactly once after you press the INIT button.
 This is where you should put all code for the OpMode.
 waitForStart():
 This method pauses the Op-Mode until you press the START button on the driver station.
 isStarted():
 returns true if the START button has been pressed, otherwise it returns false.
 isStopRequested():
 returns true if the STOP button has been pressed, otherwise it returns false.
 idle():
 calls Thread.yield, allowing other threads at the same priority level to run.
 opModeIsActive():
 returns isStarted() && !isStopRequested() and calls idle().
 opModeInInit():
 returns !isStarted() && !isStopRequested() and does not call idle().
 *****************************/

@TeleOp(name = "Mark IV.", group="Linear OpMode")
public class Mark4 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Servo grabby;
    DcMotor lift = null;
    BNO055IMU imu;

    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub

    /* TODO
    private int liftFloor = lift.setPosition(0); // Encoder value for height of lift resting on the ground, should be theoretical zero
    private int liftGround = ; Encoder value for height of ground junction and driving around
    private int liftLow = ; Encoder value for height of low junction
    private int liftMedium ; Encoder value for height of medium junction
    private int liftHigh = ; Encoder value for height of high junction
    */

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");

        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);

        lift.setTargetPosition(100); // TODO
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * Both right side motors should be going in one direction,
         * and both left side motors going in the opposite direction
         */

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // TODO lift.setDirection(DcMotorCo);
        // https://youtu.be/d0liBxZCtrA


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
            // TODO
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */

            /*
            gamepad2.left_bumper // Set lift to ground junction height
            gamepad2.dpad_down // Set lift to on-the-ground height
            gamepad2.dpad_right // Set lift top low height
            gamepad2.dpad_left // Set lift to medium height
            gamepad2.dpad_up // Set lift to high junction height
            gamepad2.a // Set claw to close position
            gamepad2.b // Set claw to open position
            gamepad2.left_trigger //  Set lift to micro positions up
            gamepad2.right_trigger //  Set lift to micro positions down
            */

            // Drives the robot forward and backwards
            double y = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
            double x = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
            double rot = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement

            double frontLeftPower = (y + x + rot);
            double backLeftPower = (y - x + rot);
            double frontRightPower = (y - x - rot);
            double backRightPower = (y + x - rot);

            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            /*
             * Telemetry Data for Driver & Optimization
             ** TODO Show the elapsed game time
             ** TODO Show wheel power output during teleop
             ** TODO Show claw-grabber position for testing
             ** TODO Show the lift motor position for testing
             */


            // lift code
            /*
            a mode, as i refer to it, is which 'mode' the lift is in, ground, low, medium, or high,
            or in number form, 0, 1, 2, and 3.
             */
            int CMode = 0; // current mode
            int DMode = 0; // desired mode
            int time = 0; // time to go up a mode, measured in milliseconds
            int mtg = DMode - CMode; // modes to go until desired mode is reached

            lift.setDirection(DcMotorSimple.Direction.FORWARD);

            if (mtg < 0){ // if the desired mode is below the current mode, set the motor direction to reverse and edit the appropriate variables
                mtg = mtg * -1;
                lift.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if (gamepad1.dpad_up){
                DMode = 3; // the desired mode is 3
                lift.setPower(1); // turns the motor on
                sleep(time*mtg); // sleep when (the amount of time for one mode*modes to go) seconds have passed
                CMode = 3; // now that we are at mode three, set the current mode to mode 3
                // the other 'dpad if statements' function the same with only the DModes and CModes differing
            }

            if (gamepad1.dpad_down){
                DMode = 0;
                lift.setPower(1);
                sleep(time*mtg);
                CMode = 0;
            }

            if (gamepad1.dpad_left){
                DMode = 1;
                lift.setPower(1);
                sleep(time*mtg);
                CMode = 1;
            }

            if (gamepad1.dpad_right){
                DMode = 2;
                lift.setPower(1);
                sleep(time*mtg);
                CMode = 2;
            }

            //leftDrive.setPower(leftPower);
            //rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L (%.2f), Front R (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Motors", "Back L (%.2f), Back R (%.2f)", backLeftPower, backRightPower);
            telemetry.update();

        }
    }
} // End of Class