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
    private Servo YSNP;
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



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        YSNP = hardwareMap.servo.get("YSNP");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(OPEN);
        YSNP.setPosition(PASS);
        lift.setTargetPosition(0);

        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        // while the operational mode is active...
        while (opModeIsActive()) {

            // if b is pressed on gamepad 2...
            if (gamepad2.b) {
                grabby.setPosition(OPEN); // set claw, or "grabby" position to open
            }
            if (gamepad2.a){
                grabby.setPosition(0); // close grabby
            }

            // Reset the lift encoder if the lift was not in a zero position at the start of teleop
            if (gamepad2.start) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            // creates the string 'liftpos' which will be modified later in if statements to show the current position of the lift on the gamepad
            String liftpos = "no input";

            // used for exact measurements by the drive team
            // while the left trigger is pushed, increase the height of the lift
            while (gamepad2.left_bumper){ // while the left bumper is being pressed on gamepad 2...
                lift.setTargetPosition(lift.getCurrentPosition()+BEE_SPEED); // set the lift's target/goal position to the current position plus "BEE_SPEED", or the set speed of the bumper
                lift.setPower(LIFT_POWER); // set the power to the set lift power
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // tells the lift to go
            }

            // while the right trigger is pushed, decrease the height of the lift
            while (gamepad2.right_bumper){
                lift.setTargetPosition(lift.getCurrentPosition()-BEE_SPEED);
                lift.setPower(LIFT_POWER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_up) { // if up is pressed on the dpad
                lift.setTargetPosition(LIFT_HIGH); // tell the robot it needs to go to LIFT_HIGH, not to actually go
                lift.setPower(LIFT_POWER); // turns on the power in the lift motor
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // tells the robot to actually go to the target position
            }
            if((LIFT_HIGH - DIPLOMAT) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + DIPLOMAT)){
                liftpos = "high"; // tell the gamepad to say "lift is at high position" instead of "no input", or whatever else it was set as
            }

            if (gamepad2.dpad_right){
                lift.setTargetPosition(LIFT_MEDIUM);
                lift.setPower(LIFT_POWER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if((LIFT_MEDIUM - DIPLOMAT) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_MEDIUM + DIPLOMAT)){
                liftpos = "medium";
            }

            if (gamepad2.dpad_left){
                lift.setTargetPosition(LIFT_LOW);
                lift.setPower(LIFT_POWER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if((LIFT_LOW - DIPLOMAT) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_LOW + DIPLOMAT)){
                liftpos = "low";
            }

            if (gamepad2.dpad_down){ // this is the "floor" height, or as low as the robot can go
                lift.setTargetPosition(LIFT_FLOOR);
                lift.setPower(LIFT_POWER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if((LIFT_FLOOR - DIPLOMAT) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_FLOOR + DIPLOMAT)){
                /*
                we originally had more use for the DIPLOMAT variable, but things changed and now we use it exclusively for the telemetry
                because we have no real reason to delete it and it makes the telemetry more accurate    `
                * */
                liftpos = "floor";
                lift.setPower(0); // this line of code is very important, if it isn't here the motor will burn up from overuse
            }

            if (gamepad2.y){ // this is the ground junction
                lift.setTargetPosition(LIFT_GROUND);
                lift.setPower(0.5);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if((LIFT_GROUND - DIPLOMAT) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_GROUND + DIPLOMAT)){
                liftpos = "ground";
            }

            telemetry.update();

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
