package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    //create motors
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;
    public Servo grabber = null;

    public final static double POS_HOME = 0.0;
    public final static double POS_MIN = -1.0;
    public final static double POS_MAX = 1.0;

    //create servo
    public Servo randomServo = null;

    //additional variables
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public Hardware(HardwareMap hwMap){
        initialize(hwMap);
    }

    public Hardware() {

    }

    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        //connect servo
        randomServo = hardwareMap.get(Servo.class, "randomServo");

        //set up motor direction
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        //set motor mode
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set zero power behavior
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motors to use no power
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        // define and initialize all installed servos
        grabber = hwMap.servo.get("grabber"); // set equal to the name in the phone
        grabber.setPosition(POS_HOME); // sets the position to the servo, upon awakening of the robot, to the home value

    }
}
