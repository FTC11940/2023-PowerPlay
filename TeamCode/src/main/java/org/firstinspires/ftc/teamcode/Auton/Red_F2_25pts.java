/*
 */

package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Red-25 pts", group = "Robot")
// @Disabled

public class Red_F2_25pts extends LinearOpMode {


    Servo grabby;
    Servo YSNP;
    DcMotor lift;
    TouchSensor touchy;
    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // Declare IMU and variables
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;

    /*
    These variable are declared here (as class members) so they can be updated in various methods,
    but still be displayed by sendTelemetry()
     */
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // Match our TeleOp file
        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(CLOSED); // Needs to be closed at start of Auton

        YSNP = hardwareMap.servo.get("YSNP");
        YSNP.setPosition(PASS); // Needs to be closed at start of Auton

        lift = hardwareMap.get(DcMotor.class, "lift");

        // lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        touchy = hardwareMap.get(TouchSensor.class, "touchy");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Match our TeleOp file
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Stop and reset all drive motors
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Brake all drive motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        /*  Set the encoders for closed loop speed control,
            and reset the heading for all drive motors
         */
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetHeading();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        /*

        /* REFERENCE
        // driveStraight(DRIVE_SPEED, 10.0, 45.0);  // action - e.g. turn 45 Degrees to the left
        // turnToHeading( TURN_SPEED,  -15.0);      // action - turn 15 degrees to the right
        // holdHeading( TURN_SPEED,  0.0, 0.5);     // action - hold last heading for a 1/2 second
        */


        waitForStart();

        // Drive towards the high junction
        driveStraight(DRIVE_SPEED, 52.0, 0.0); //

        // Lift code up high
        lift.setTargetPosition(2850);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_HIGH - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + TOLERANCE)) {
            telemetry.addData("Lift High Status", "You've arrived at your HIGH destination");
            // lift.setPower(0);
        }


        turnToHeading(TURN_SPEED, -39.0);

        YSNP.setPosition(SHUT); // Lift need to be up before shutting YSNP
        while (opModeIsActive()) {
            if (touchy.isPressed()) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                sleep(500);
                grabby.setPosition(OPEN);
                break;
            } else {
                grabby.setPosition(CLOSED);
                frontLeftMotor.setPower(0.2);
                backLeftMotor.setPower(0.2);
                frontRightMotor.setPower(0.2);
                backRightMotor.setPower(0.2);
            }

        }
        YSNP.setPosition(PASS);
        driveStraight(DRIVE_SPEED, -8.0, 0.0); //


        // TODO Just gonna put this here for fun testing later :)
        // strafeRightShiny(-480); // Made it negative so it goes left rather write a left for now
        turnToHeading(TURN_SPEED, 90.0);
        lift.setTargetPosition(LIFT_FIVE_STACK);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_FIVE_STACK - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_FIVE_STACK + TOLERANCE));


        driveStraight(DRIVE_SPEED, 27.5, 0.0); // Drive to substation

        grabby.setPosition(CLOSED);
        sleep(650);

        lift.setTargetPosition(2950); // FIXME? 2850
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_HIGH - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + TOLERANCE)) {
            telemetry.addData("Lift High Status", "You've arrived at your destination");
        }

        driveStraight(DRIVE_SPEED, -27.5, 0.0); // Drive to substation
        turnToHeading(TURN_SPEED, -39.0);

        YSNP.setPosition(SHUT); // Lift need to be up before shutting YSNP
        while (opModeIsActive()) {
            if (touchy.isPressed()) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                sleep(700);
                grabby.setPosition(OPEN);
                break;
            } else {
                grabby.setPosition(CLOSED);
                frontLeftMotor.setPower(0.2);
                backLeftMotor.setPower(0.2);
                frontRightMotor.setPower(0.2);
                backRightMotor.setPower(0.2);
            }

        }

        YSNP.setPosition(PASS);
        driveStraight(DRIVE_SPEED, -8.0, 0.0); //
        turnToHeading(TURN_SPEED, 90.0);
        // Insert lift code down here
        lift.setTargetPosition(LIFT_FOUR_STACK);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_FOUR_STACK - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_FOUR_STACK + TOLERANCE)) {
            telemetry.addData("Lift 3/4 Stack Status", "You've arrived at your 3/4 cone destination");
        }

        driveStraight(DRIVE_SPEED, 27.5, 0.0); //

        grabby.setPosition(CLOSED);
        sleep(650);

        // Lift code up high
        lift.setTargetPosition(2950);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_HIGH - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + TOLERANCE)) {
            telemetry.addData("Lift High Status", "You've arrived at your HIGH destination");
            //lift.setPower(0);
        }

        driveStraight(DRIVE_SPEED, -27.5, 0.0);
        turnToHeading(TURN_SPEED, -39.0);
        driveStraight(DRIVE_SPEED, 7.0, 0.0); //
        sleep(650);
        grabby.setPosition(OPEN);
        driveStraight(DRIVE_SPEED, -8.0, 0.0); //
        turnToHeading(TURN_SPEED, 0.0);
        grabby.setPosition(CLOSED);
        // Lift code down
        lift.setTargetPosition(LIFT_GROUND);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Test the telemetry statement before setting power to zero.
        if ((LIFT_GROUND - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_GROUND + TOLERANCE)) {
            telemetry.addData("Lift Ground Status", "You've arrived at your GROUND destination");
        }



        // TODO Backup to Position Two tile?


        /*
         * TODO Write specific parking code in the methods of this file (near botttom)
         * Comment out or uncomment as needed for testing purposes
         */

        // TODO Test before adding in vision

        //signalParkOne();
        // Strafe Left

        // TODO Test before adding in vision
         signalParkTwo();
        // Should already be in position by default just by backing up after last cone


        // TODO Test before adding in vision
        // signalParkThree();
        // Strafe Right

        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);  // Pause to display last telemetry message.

    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * DRIVE_COUNTS_PER_INCH);
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
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", frontLeftTarget, frontRightTarget);
            telemetry.addData("Target Pos L:R", "%7d:%7d", backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    /*
     * Functions to run for autonomous parking
     * These functions should be used to replace specific code in the logic testing
     * which should make it make it more reusable in various autonomous files
     * without messing with the logic */

    private void signalParkOne() {
       strafeLeft();
       sleep(300);
        driveStraight(DRIVE_SPEED,-12.0,0);
    }

    private void signalParkTwo() {
        driveStraight(DRIVE_SPEED,-12.0,0);
    }

    private void signalParkThree() {
        strafeRight();
        sleep(300);
        driveStraight(DRIVE_SPEED,-12.0,0);
    }

    private void strafeLeft() {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftMotor.setPower(-1.0); // (-) strafe left
        backRightMotor.setPower(-1.0); // (-) strafe left

        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);

        while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * -480))
            ; // (-) strafe left
        while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * -480))
            ; // (-) strafe left

        while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * 480)) ;
        while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED * 480)) ;
    }

    private void strafeRight() {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftMotor.setPower(1.0); // (-) strafe left
        backRightMotor.setPower(1.0); // (-) strafe left

        frontRightMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);

        while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * 480))
            ; // (-) strafe left
        while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * 480))
            ; // (-) strafe left

        while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * -480))
            ;  // (-) strafe right
        while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED * -480))
            ;
    }


    private void strafeRightShiny(int desiredEncoder) {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(1.0); // (-) strafe left
        backRightMotor.setPower(1.0); // (-) strafe left

        frontRightMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);

        while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder))
            ; // (-) strafe left
        while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder))
            ; // (-) strafe left

        while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * -desiredEncoder))
            ;  // (-) strafe right
        while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED * -desiredEncoder))
            ;

        // TODO Test to see if this using encoder instead of time
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    } // end Shiny Right
}

/* Copyright (c) 2022 FIRST. All rights reserved.
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