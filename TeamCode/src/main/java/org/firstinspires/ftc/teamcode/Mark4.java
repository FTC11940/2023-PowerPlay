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

    // For Lift
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

        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);

        // lift.setTargetPosition(0);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * Both right side motors should be going in one direction,
         * and both left side motors going in the opposite direction
         */

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD); // FIXME
        // https://youtu.be/d0liBxZCtrA

        // TODO


        int liftFloor = (0); // Encoder value for height of lift resting on the ground, should be theoretical zero
        int liftGroundAuto = (100); // Encoder value for height of ground junction and driving around

        double newLiftGround = (COUNTS_PER_INCH*1); // Encoder value for height of ground junction and driving around
        double newLiftLow = (COUNTS_PER_INCH*6); // Encoder value for height of low junction
        double newLiftMedium = (COUNTS_PER_INCH*12); // Encoder value for height of medium junction
        double newLiftHigh = (COUNTS_PER_INCH*18); // Encoder value for height of high junction

        int liftGround = (int)newLiftGround; // Encoder value for height of ground junction and driving around
        int liftLow = (int)newLiftLow; // Encoder value for height of low junction
        int liftMedium = (int)newLiftMedium; // Encoder value for height of medium junction
        int liftHigh = (int)newLiftHigh; // Encoder value for height of high junction

        waitForStart();
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            gamepad2.left_bumper  // Set lift to ground junction height
            gamepad2.dpad_down // Set lift to on-the-ground height
            gamepad2.dpad_right // Set lift top low height
            gamepad2.dpad_left // Set lift to medium height
            */

            // Set lift to high junction height
            if (gamepad1.dpad_up) {
                lift.setTargetPosition(liftHigh);
            }
            /*
            gamepad2.a // Set claw to close position
            gamepad2.b // Set claw to open position
            gamepad2.left_trigger //  Set lift to micro positions up
            gamepad2.right_trigger //  Set lift to micro positions down
            */
            lift.setTargetPosition(liftGround);// FIXME This should lift to the Ground position upon start
            lift.setPower(0.25); // FIXME Set power output of lift
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (lift.isBusy()) {
                telemetry.addData("Status","Running lift to Ground Position");
                telemetry.update();
            }
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Move lift back
            /*
            newTarget = lift.getTargetPosition() - (int)
             */

            //Drives the robot forward and backwards
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

            //leftDrive.setPower(leftPower);
            //rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L (%.2f), Front R (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Motors", "Back L (%.2f), Back R (%.2f)", backLeftPower, backRightPower);
            telemetry.update();

        }
    }
} // End of Class