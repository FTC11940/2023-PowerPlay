package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name = "Mark VI.", group="Linear OpMode")
// @Disabled
public class Mark6 extends LinearOpMode {

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

    static final double TICK_COUNT = 1000;



    @Override
    public void runOpMode() throws InterruptedException {

        // robot.init(Hardware);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");


        // TODO string to hardware map for lift
        lift = hardwareMap.get(DcMotor.class, "lift");


        // use braking $coy slow the motor down
        // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        // lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(0.5);

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

        lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        double LIFT_LOW = TICK_COUNT;

        // FIXME
        lift.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        lift.setTargetPosition((int)LIFT_LOW); // TODO Moved to be after stop and reset
        lift.setPower(0.5); // FIXME Changing this power output had no impact
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);



        // Feedback for what the motor is doing
        while (lift.isBusy()) {
            // Send telemetry info to dashboard
            telemetry.addData("Status", "Running motor to LIFT LOW");
            telemetry.update();
        }

        // lift.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        lift.setTargetPosition(0); // TODO Moved to be after stop and reset
        lift.setPower(0.2); // FIXME Changing this power output had no impact
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Feedback for what the motor is doing
        while (lift.isBusy()) {
            // Send telemetry info to dashboard
            telemetry.addData("Status", "Running motor to Zero");
            telemetry.update();
        }

        // No longer busy so turn off the lift
        lift.setPower(0);

        if (isStopRequested()) return;

        telemetry.update();


    }
} // End of Class