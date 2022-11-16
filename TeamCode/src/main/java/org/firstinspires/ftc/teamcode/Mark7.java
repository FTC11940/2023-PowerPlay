/*
 * Mark VII. is an autonomous like program that should close the claw around a cone,
 * rise to a preset height (encoder value), drop the cone, and pick up another
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Mark VII.", group="Linear OpMode")
@Disabled

public class Mark7 extends LinearOpMode {

    // Reference the hardware map file
    // Hardware robot = new Hardware();

    Servo grabby;
    DcMotor lift;

    private ElapsedTime runtime = new ElapsedTime();

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // robot.init(H
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        grabby = hardwareMap.servo.get("grabby");

        lift = hardwareMap.get(DcMotor.class, "lift");


        // use braking to slow the motor down
        // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        // lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(OPEN);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * Both right side motors should be going in one direction,
         * and both left side motors going in the opposite direction
         */

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        /*
        * This is the proper sequence for functional lift code.
        */

        // Grab Cone 1
        grabby.setPosition(CLOSED);

        lift.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER)); // Only needed for the first lift code sequence
        lift.setTargetPosition(LIFT_LOW);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drop Cone 1
        grabby.setPosition(OPEN);
        sleep(1000); // Pause for before moving back. Not needed in teleop code

        // Feedback for what the motor is doing
        while (lift.isBusy()) {
            // Send telemetry info to dashboard
            telemetry.addData("Status", "Running motor to LIFT LOW");
            telemetry.update();
        }

        // Set lift down for Auton to grab another cone
        lift.setTargetPosition(0);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(CLOSED); // Grab Cone 2
        sleep(500); // Pause for before moving back.

        lift.setTargetPosition(LIFT_GROUND);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(OPEN); // Drop Cone 2
        sleep(1000); // Pause for before moving back. Not needed in teleop code

        // Set lift down for Auton to grab another cone
        lift.setTargetPosition(0);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(CLOSED); // Grab Cone 3
        sleep(500); // Pause for before moving back.

        lift.setTargetPosition(LIFT_MEDIUM);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(OPEN); // Drop Cone 3
        sleep(1000); // Pause for before moving

        // Set lift down for Auton to grab another cone
        lift.setTargetPosition(0);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(CLOSED); // Grab Cone 4
        sleep(500); // Pause for before moving.

        lift.setTargetPosition(LIFT_HIGH);
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(OPEN); // Drop Cone 4
        sleep(1000); // Pause for before moving back. Not needed in teleop code

        // Set lift down for Auton to grab another cone
        lift.setTargetPosition(0);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabby.setPosition(CLOSED); // Grab Cone 5
        sleep(500); // Pause for before moving back.

        // Feedback for what the motor is doing
        while (lift.isBusy()) {
            // Send telemetry info to dashboard
            telemetry.addData("Status", "Running motor to zero position");
            telemetry.update();
        }

        // No longer busy so turn off the lift
        lift.setPower(0);

        if (isStopRequested()) return;

        telemetry.update();

    }
} // End of Class