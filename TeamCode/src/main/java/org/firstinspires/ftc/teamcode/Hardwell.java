package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.LIFT_FLOOR;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardwell {

    private LinearOpMode opForce = null;

    // Create motors
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor lift = null; // creates the lift motor
    private Servo grabby = null; // creates the claw servo
    private ColorSensor color = null; // create the color sensor
    private DistanceSensor distance = null; // creates the distance sensor

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardwell (LinearOpMode opmode) {
        opForce = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftMotor = opForce.hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor = opForce.hardwareMap.get(DcMotor.class,"frontRightMotor");
        backLeftMotor = opForce.hardwareMap.get(DcMotor.class,"backLeftMotor");
        backRightMotor = opForce.hardwareMap.get(DcMotor.class,"backRightMotor");
        lift = opForce.hardwareMap.get(DcMotor.class,"lift");

        // Set up motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        // Run motors using encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop and reset encoders on initialization
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to BRAKE rather than FLOAT (coasting) behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to use no power at start so they don't take off on initialization
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        lift.setPower(0);

        // Define and initialize installed servos.
        grabby = opForce.hardwareMap.get(Servo.class, "grabby");

        // Starting positions
        grabby.setPosition(OPEN);
        lift.setTargetPosition(LIFT_FLOOR);

        BNO055IMU imu;


    } // end of init

} // End class