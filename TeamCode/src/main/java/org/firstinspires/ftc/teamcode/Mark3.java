package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/****************************
    runOpMode(): Code inside this method will run exactly once after you press the INIT button. This is where you should put all code for the OpMode.
    waitForStart(): This method pauses the Op-Mode until you press the START button on the driver station.
    isStarted(): returns true if the START button has been pressed, otherwise it returns false.
    isStopRequested(): returns true if the STOP button has been pressed, otherwise it returns false.
    idle(): calls Thread.yield, allowing other threads at the same priority level to run.
    opModeIsActive(): returns isStarted() && !isStopRequested() and calls idle().
    opModeInInit(): returns !isStarted() && !isStopRequested() and does not call idle().
 *****************************/

@TeleOp(name = "Mark III.", group="Linear OpMode")
@Disabled

public class Mark3 extends LinearOpMode {

    Servo grabby;

    private ElapsedTime runtime = new ElapsedTime();

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);

        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        // Set motor directions
        // Both right side motors going same direction
        // Both left side motors going same direction, but coded as opposite of the right
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            // Controller Mapping
            if (gamepad1.y) {
                grabby.setPosition(0.5);
            }
            if (gamepad1.a){
                grabby.setPosition(0);
            }

            telemetry.update();


            // Drives the robot forward and backwards
            double y = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
            double x = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
            double rot = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement

            /* This may not be optimal. Consider using
            // Uses the left thumbstick for forward & backwards robot movement
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */

            // input: theta and power
            double theta = Math.atan2(y,x);
            double power = Math.hypot(x,y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            // Variables for wheel motor power and inputs
            double frontLeftPower = (power * cos/max + rot);
            double frontRightPower = (power * sin/max - rot);
            double backLeftPower = (power * sin/max + rot);
            double backRightPower = (power * cos/max - rot);

            if ((power + Math.abs(rot)) > 1) {
                frontLeftPower /= power + rot;
                frontRightPower /= power + rot;
                backLeftPower /= power + rot;
                backRightPower /= power + rot;
            }

            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            /*
             * Telemetry Data for Driver & Optimization
             ** Show the elapsed game time
             ** Show wheel power output during teleop TODO test to see if it works properly
             ** TODO Show claw-grabber position for testing
             ** TODO Show the lift motor position for testing
             */

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front left (%.2f), Front right (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Motors", "Back left (%.2f), Back right (%.2f)", backLeftPower, backRightPower);
            telemetry.update();

        }
    }
} // End of Class