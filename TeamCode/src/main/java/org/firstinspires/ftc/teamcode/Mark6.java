package org.firstinspires.ftc.teamcode;

// FIXME Don't know if this is best practice. Ask Maasser
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.*;

import static org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Mark VI.", group="Linear OpMode")
@Disabled
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

    @Override
    public void runOpMode() throws InterruptedException {

        // robot.init(Hardware);

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

        // Only needed for the first lift code sequence
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setTargetPosition(LIFT_MEDIUM);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.25);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(2000);

        // Pause for before moving back. Not needed in teleop code
        // Feedback for what the motor is doing

        /*
        while (lift.isBusy()) {
            // Send telemetry info to dashboard
            telemetry.addData("Status", "Running motor to LIFT GROUND");
            telemetry.update();
        }
        */


        lift.setTargetPosition(0);
        lift.setPower(0.25);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000); // Pause for before moving back. Not needed in teleop code


        // No longer busy so turn off the lift
        lift.setPower(0);

        if (isStopRequested()) return;

        telemetry.update();

    }
} // End of Class
