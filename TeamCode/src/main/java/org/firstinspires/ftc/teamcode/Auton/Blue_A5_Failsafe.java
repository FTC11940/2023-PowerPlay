/**
 * Blue Failsafe 1 Auton starts in A5 and drives south to drop a cone and park in terminal A4
 */

package org.firstinspires.ftc.teamcode.Auton;
import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Blue-A5 Failsafe", group="Robot")
// Disabled

public class Blue_A5_Failsafe extends LinearOpMode {


    Servo grabby;
    DcMotor lift;
    // Declare OpMode members
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    // Declare IMU and variables
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU
    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    /*
    These variable are declared here (as class members) so they can be updated in various methods,
    but still be displayed by sendTelemetry()
     */
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     frontLeftTarget    = 0;
    private int     frontRightTarget   = 0;
    private int     backLeftTarget = 0;
    private int     backRightTarget =0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;   // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;   // Max Turn speed to limit turn rate

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // Match our TeleOp file
        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(0.0); // Needs to be closed at start of Auton
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");

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

        * AUTON NAME: Blue FailSafe 1
        * REFERENCE
        // driveStraight(DRIVE_SPEED, 10.0, 45.0);  // action - e.g. turn 45 Degrees to the left
        // turnToHeading( TURN_SPEED,  -15.0);      // action - turn 15 degrees to the right
        // holdHeading( TURN_SPEED,  0.0, 0.5);     // action - hold last heading for a 1/2 second
        */


        // Autonomous failsafe Blue A5
        driveStraight(DRIVE_SPEED, 2.0, 0.0); // Drive forward to get off the wall
        lift.setTargetPosition(LIFT_LOW);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_LOW - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_LOW + TOLERANCE)) {
            telemetry.addData("Lift Low Status", "You've arrived at your HIGH destination");
            // lift.setPower(0);
        }
        turnToHeading( TURN_SPEED,  -44.0);//Turn 90 to face direction of terminal
        driveStraight(DRIVE_SPEED, 4.0, 0.0); //
        grabby.setPosition(OPEN);
        driveStraight(DRIVE_SPEED, -4.0, 0.0); //
        lift.setTargetPosition(LIFT_GROUND);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_GROUND - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_GROUND + TOLERANCE)) {
            telemetry.addData("Lift Low Status", "You've arrived at your HIGH destination");
            // lift.setPower(0);
        }
        turnToHeading( TURN_SPEED,  -90.0);//Turn 90 to face direction of terminal
        driveStraight(DRIVE_SPEED, 14.00, 0.0); // Drive to terminal
        turnToHeading( TURN_SPEED,  0.0); // Turn back to face forward
        driveStraight(DRIVE_SPEED, -1.00, 0.0); // park
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



    //maxDriveSpeed == MAX Speed for forward/reverse motion (range 0.0 to positive 1.0) .
    //distance == Distance (in inches) to move from current position.  Negative distance means move backward.
    //heading == Absolute Heading Angle (in Degrees) relative to last gyro reset.

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * FAILSAFE_COUNTS_PER_INCH);
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

            maxDriveSpeed = Math.abs(maxDriveSpeed); // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            moveRobot(maxDriveSpeed, 0); // Start driving straight, and then enter the control loop


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


    // maxTurnSpeed == Desired MAX speed of turn. (range 0 to +1.0)
    /* heading == Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */


    // spins on central axis to point in a new direction
    // will stop if/when it reaches the desired angle or the driver ends the OpMode
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


    // maxTurnSpeed === Maximum differential turn speed (range 0 to +1.0)
     /* heading === Absolute Heading Angle (in Degrees) relative to last gyro reset.
                       0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
                       If a relative angle is required, add/subtract from current heading. */
    // holdTime   Length of time (in seconds) to hold the specified heading.

    // holds and obtains a heading until the requested time amount has elapsed
    // used for giving a robot a minute to stabilize it's heading between movements
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


    // desiredHeading  ===  The desired absolute heading (relative to last heading reset)
    // proportionalGain  ===  Gain factor applied to heading error to obtain turning power.
    // returns:                      Turning power needed to get to required heading.

    // This method uses a Proportional Controller to determine how much steering correction is required.
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    //drive === forward motor speed
    //turn === clockwise turning motor speed.

    // takes the separate drive (fwd/rev) and turn (right/left) requests and combines them
    // then it gives the appropriate speed commands to the wheel motors
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        backLeftMotor.setPower(leftSpeed);
        backRightMotor.setPower(rightSpeed);
    }


    //straight === Set to true if we are driving straight, and the encoder positions should be included in the telemetry.

    //Display the various control parameters while driving
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      frontLeftTarget,  frontRightTarget);
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      backLeftTarget,  backRightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }


    // read the raw (un-offset Gyro heading) directly from the IMU
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // set the off-set heading back to zero
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
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