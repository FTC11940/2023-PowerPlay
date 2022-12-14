package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.BEE_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.DIPLOMAT;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_FLOOR;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_GROUND;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_LOW;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_MEDIUM;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_POWER;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;
import static org.firstinspires.ftc.teamcode.Constants.PASS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

// @Disabled
public class TouchyAction extends LinearOpMode {

    private Servo grabby;
    private Servo YSNP;
    private DcMotor lift;
    TouchSensor touchy;
    TouchSensor touchy2;

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
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        YSNP = hardwareMap.servo.get("YSNP");

        // Set starting position of the grabby claw. 0.5 is open, 0.0 is closed
        grabby.setPosition(OPEN);
        YSNP.setPosition(PASS);
        lift.setTargetPosition(0);

        touchy = hardwareMap.get(TouchSensor.class, "touchy");
        touchy2 = hardwareMap.get(TouchSensor.class, "touchy2");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        // while the operational mode is active...

        while (opModeIsActive()) {

        /*
        Robot drops the closes the and drives forward until the button, touchy, is pressed.
        This sets the correct distance/depth from the terminal pole
         */

            YSNP.setPosition(CLOSED);

            while (opModeIsActive()) {
                if (touchy.isPressed()) {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                    sleep(700);

                    break;
                } else {
                    grabby.setPosition(CLOSED);
                    frontLeftMotor.setPower(0.2);
                    backLeftMotor.setPower(0.2);
                    frontRightMotor.setPower(0.2);
                    backRightMotor.setPower(0.2);
                }

            }

        /*
        Robot raises the gate
         */

            YSNP.setPosition(PASS);

            while (opModeIsActive()) {
                if (touchy2.isPressed()) {
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                    sleep(250);
                    frontLeftMotor.setPower(-0.3);
                    backLeftMotor.setPower(-0.3);
                    frontRightMotor.setPower(0.3);
                    backRightMotor.setPower(0.3);
                    sleep(150);
                    grabby.setPosition(OPEN);
                    break;
                } else {
                    grabby.setPosition(CLOSED);
                    frontLeftMotor.setPower(0.2);
                    backLeftMotor.setPower(0.2);
                    frontRightMotor.setPower(-0.2);
                    backRightMotor.setPower(-0.2);
                }

            }


        }
    }
}// End of Class
