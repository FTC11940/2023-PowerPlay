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

@TeleOp(name = "Mecanum", group="Linear OpMode")
// @Disabled
public class Mark5 extends LinearOpMode {

    Servo grabby;
    DcMotor lift;
    private ElapsedTime runtime = new ElapsedTime();

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub
    BNO055IMU imu;

    // for lift
    static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    static final double PULLEY_DIAMETER_INCHES = 2.0 ; // For figuring circumference
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);

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

        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // FIXME - Scoy added, I think this is needed. Had to use something like this in Auton when using the encoder
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        // TODO add some telemetry.addData stuff
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            if (gamepad1.b) {
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
            int time = 2000; // time to go up a mode, measured in milliseconds
            int mtg = DMode - CMode; // modes to go until desired mode is reached
            int power = 1;

            lift.setPower(0);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //lift.setDirection(DcMotorSimple.Direction.FORWARD);

            //lift.setTargetPosition(0);
            // FIXME
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.y) {
                lift.setTargetPosition(1000);
                lift.setPower(0.2);
            }
            if (gamepad1.x){
                lift.setTargetPosition(50);
                lift.setPower(0.2);
                // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L (%.2f), Front R (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Motors", "Back L (%.2f), Back R (%.2f)", backLeftPower, backRightPower);
            telemetry.update();

        }
    }
} // End of Class