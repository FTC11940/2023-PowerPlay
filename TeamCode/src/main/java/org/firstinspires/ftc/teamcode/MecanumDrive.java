package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

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
public class MecanumDrive extends LinearOpMode {

    private Servo grabby;
    private DcMotor lift;
    private ElapsedTime runtime = new ElapsedTime();

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub
    BNO055IMU imu;

    static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 4.01575 ; // For figuring circumference
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget =0;

    public void strafeDrive (double maxDriveSpeed,
                             double distance
                             ){
        while (opModeIsActive()){
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;
            backRightTarget = backRightMotor.getCurrentPosition() + moveCounts;

            // This block was commented out
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftMotor.setTargetPosition(frontLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            backRightMotor.setTargetPosition(backRightTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            maxDriveSpeed = Math.abs(maxDriveSpeed);

        };
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(grabby_open);
        lift.setTargetPosition(0);

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

        //if (isStopRequested()) return;
        while (opModeIsActive()) {

            if (gamepad2.b) {
                grabby.setPosition(grabby_open);
            }
            if (gamepad2.a){
                grabby.setPosition(0);
            }

            if (gamepad1.right_bumper){
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftMotor.setTargetPosition(hornet_speed);
                backLeftMotor.setTargetPosition(-hornet_speed);
                frontRightMotor.setTargetPosition(-hornet_speed);
                backRightMotor.setTargetPosition(hornet_speed);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setPower(1);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(1);
                backRightMotor.setPower(1);

            }

            if (gamepad1.left_bumper){
                frontLeftMotor.setPower(1);
                frontRightMotor.setPower(1);
                backLeftMotor.setPower(1);
                backRightMotor.setPower(1);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                strafeDrive(1,36.0);
            }

            String liftpos = "no input";

            // while the left trigger is pushed, increase the height of the lift
            // used for exact measurements by the drive team
            while (gamepad2.left_bumper){
                lift.setTargetPosition(lift.getCurrentPosition()+bee_speed);
                lift.setPower(lift_power);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // while the right trigger is pushed, decrease the height of the lift
            while (gamepad2.right_bumper){
                lift.setTargetPosition(lift.getCurrentPosition()-bee_speed);
                lift.setPower(lift_power);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_up) { // if up is pressed on the dpad
                lift.setTargetPosition(lift_high); // tell the robot it needs to go to lift_high, not to actually go
                lift.setPower(lift_power); // turns on the power in the lift motor
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // tells the robot to actually go to the target position
            }
            if((lift_high - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_high + diplomat)){
                liftpos = "high"; // tell the gamepad to say "lift is at high position"
            }

            if (gamepad2.dpad_right){
                lift.setTargetPosition(lift_mid);
                lift.setPower(lift_power);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if((lift_mid - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_mid + diplomat)){
                liftpos = "medium";
            }

            if (gamepad2.dpad_left){
                lift.setTargetPosition(lift_low);
                lift.setPower(lift_power);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if((lift_low - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_low + diplomat)){
                liftpos = "low";
            }

            if (gamepad2.dpad_down){
                lift.setTargetPosition(lift_floor);
                lift.setPower(lift_power);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if((lift_floor - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_floor + diplomat)){
                /*
                we originally had more use for the diplomat variable, but things changed and now we use it exclusively for the telemetry
                because we have no real reason to delete it and it makes the telemetry more accurate    `
                * */
                liftpos = "floor";
            }

            if (gamepad2.y){
                lift.setTargetPosition(lift_ground);
                lift.setPower(0.5);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //while (lift.isBusy()){liftpos = "running to ground";}
            }

            if((lift_ground - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_ground + diplomat)){
                liftpos = "ground";
            }

            telemetry.update();

            /* This may not be optimal. Consider using
            // Uses the left thumbstick for forward & backwards robot movement

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */

            /*
            gamepad2.left_bumper // Set lift to ground junction height
            gamepad2.dpad_down // Set lift to on-the-ground height
            gamepad2.dpad_right // Set lift to low height
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
            dpaddown = ground
            dpadleft = low
            dpadright = medium
            dpad up = high
             */





            telemetry.addData("Lift Position: ", liftpos);
            telemetry.addData("Exact Lift Position", lift.getCurrentPosition());
            telemetry.addData("Status", "target position: " + lift.getTargetPosition());
            telemetry.addData("Status", "controller: " + lift.getController().toString());
            telemetry.addData("Status", "controller: " + lift.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L (%.2f), Front R (%.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Motors", "Back L (%.2f), Back R (%.2f)", backLeftPower, backRightPower);
            telemetry.update(); // this is very important! without putting this code at the end of your telemetry, your telemetry will not update with new information


        }
    }
}
// End of Class