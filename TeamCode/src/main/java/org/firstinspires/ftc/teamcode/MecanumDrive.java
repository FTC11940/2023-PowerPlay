package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    //    // Located in the Hardware file and matches with the Drive Hub robot settings
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);
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
            dpaddown = ground
            dpadleft = low
            dpadright = medium
            dpad up = high
             */

            String liftpos = "no input";

            if (gamepad1.dpad_up) { // if up is pressed on the dpad
                while (lift.isBusy()){liftpos = "running to high";} // tell the gamepad to say "lift is running to high"
                lift.setTargetPosition(lift_high); // tell the robot to go to lift position high
                lift.setPower(1); // turns on the power in the lift motor
                //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); is supposed to stop sending power to motor after the motor has finished its tasks, but does not work
            }

            if((lift_high - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_high + diplomat)){
                /* ^
                   | if the lift's current position is greater than the low position minus 10 and is less than the low position plus 10 execute the following code
                   we are doing this instead of just checking if the lift's current position is equal to the low position because robots are neurotic perfectionists who,
                   even if they are only one unit away from the lift low position, will continue to jerkily move after the command has been given in an attempt to get to the
                   exact lift low position. this could be hard for the drivers to work around, so we use complex if statements instead of simple ones
                */
                liftpos = "low"; // tell the gamepad to say "lift is at low position"
                lift.setPower(0); // stops the robot from purposeless neurotic twitching after all tasks have been fulfilled
            }

            if (gamepad1.dpad_right){
                while (lift.isBusy()){liftpos = "running to medium";}
                lift.setTargetPosition(lift_mid);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if((lift_mid - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_mid + diplomat)){
                liftpos = "medium";
                lift.setPower(0);
            }

            if (gamepad1.dpad_left){
                while (lift.isBusy()){liftpos = "running to low";}
                lift.setTargetPosition(lift_low);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if((lift_low - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_low + diplomat)){
                liftpos = "low";
                lift.setPower(0);
            }

            if (gamepad1.dpad_down){
                while (lift.isBusy()){liftpos = "running to ground";}
                lift.setTargetPosition(lift_ground);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if((lift_ground - diplomat) < lift.getCurrentPosition() && lift.getCurrentPosition() < (lift_ground + diplomat)){
                liftpos = "ground";
                lift.setPower(0);
            }



            telemetry.addData("Lift Position: ", liftpos);
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