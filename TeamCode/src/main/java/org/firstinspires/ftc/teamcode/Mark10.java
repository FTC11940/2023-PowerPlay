/*
* Mark VIII. is a teleop design to have functional drive code, claw code, and preset lift
* positions assigned to buttons
* */
package org.firstinspires.ftc.teamcode;

// FIXME Don't know if this is best practice. Ask Maasser

import static org.firstinspires.ftc.teamcode.Constants.CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_GROUND;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_LOW;
import static org.firstinspires.ftc.teamcode.Constants.LIFT_MEDIUM;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;
import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mark X.", group="Linear OpMode")
// @Disabled

public class Mark10 extends LinearOpMode {

    // Reference the hardware map file
    // Hardware robot = new Hardware();
    Servo grabby;
    DcMotor lift;
    BNO055IMU imu;
    ColorSensor color;
    DistanceSensor distance;

    private ElapsedTime runtime = new ElapsedTime();

    // Located in the Hardware file and matches with the Drive Hub robot settings
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub

    // Lift encoder conversions
    static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    static final double PULLEY_DIAMETER_INCHES = 2.0 ; // For figuring circumference
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);

    // private DistanceSensor sensorRange;


    @Override
    public void runOpMode() throws InterruptedException {


        // TODO Added Distance Sensor
        // you can use this as a regular DistanceSensor.
        // sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");

        lift = hardwareMap.get(DcMotor.class, "lift");

        color = hardwareMap.get(ColorSensor.class, "color");


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

        // Only needed when initialized.
        lift.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Assign actions to buttons

            if (gamepad1.b) {
                grabby.setPosition(OPEN);
            }

            if (gamepad1.a) {
                grabby.setPosition(CLOSED);
            }

            /* TODO
            * Code would likely be very similar to drive code because trigger is analog input
            * Input would need to keep adding power while held as well

            //  Set lift to micro positions DOWN when held
            gamepad2.left_trigger

            //  Set lift to micro positions down
            gamepad2.right_trigger

            */

            // Drives the robot forward and backwards
            double y = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
            double x = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
            double rot = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement

            double frontLeftPower = (y + x + rot);
            double backLeftPower = (y - x + rot);
            double frontRightPower = (y - x - rot);
            double backRightPower = (y + x - rot);

            // Variable to hold the current lift position
            int liftPos = lift.getCurrentPosition();


            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Grab the starting cone
            grabby.setPosition(CLOSED);

            // Testing Purposes
            if (gamepad1.y) {
                lift.setTargetPosition(LIFT_LOW);
                lift.setPower(0.5);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (lift.isBusy()) {
                    // Send telemetry info to dashboard
                    telemetry.addData("Status", "Running lift to LOW");
                    telemetry.addData("Lift Power", lift.getPower());

                    telemetry.update();
                }
                // Test the telemetry statement before setting power to zero.
                if ((LIFT_LOW - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_LOW + TOLERANCE)) {
                    telemetry.addData("Lift Low Status", "You've arrived at your Low destination");
                    lift.setPower(0);
                }
            }

            // Sets and holds lift to Ground Junction height
            if (gamepad1.start) {
                lift.setTargetPosition(LIFT_GROUND);
                lift.setPower(0.25);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (lift.isBusy()) {
                    // Send telemetry info to dashboard
                    telemetry.addData("Status", "Running lift GROUND");
                    telemetry.addData("Lift Power", lift.getPower());

                    telemetry.update();
                }

                // Test the telemetry statement before setting power to zero.
                if ((LIFT_LOW - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_LOW + TOLERANCE)) {
                    telemetry.addData("Lift Low Status", "You've arrived at your GROUND destination");
                    lift.setPower(0);
                }

            }

            // Sets and hold lift to Low Junction height
            if (gamepad1.dpad_left) {
                lift.setTargetPosition(LIFT_LOW);
                lift.setPower(0.5);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (lift.isBusy()) {
                    // Send telemetry info to dashboard
                    telemetry.addData("Status", "Running lift to LOW");
                    telemetry.addData("Lift Power", lift.getPower());
                    telemetry.update();
                }
                // Test the telemetry statement before setting power to zero.

                if ((LIFT_LOW - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_LOW + TOLERANCE)) {
                    telemetry.addData("Lift Low Status", "You've arrived at your LOW destination");
                    telemetry.addData("Lift Power", lift.getPower());
                    lift.setPower(0);
                }


                // Sets and hold lift to Medium Junction height
                else if (gamepad1.dpad_right) {
                    lift.setTargetPosition(LIFT_MEDIUM);
                    lift.setPower(0.5);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (lift.isBusy()) {
                        // Send telemetry info to dashboard
                        telemetry.addData("Status", "Running lift MEDIUM");
                        telemetry.addData("Lift Power", lift.getPower());

                        telemetry.update();
                    }

                    // Test the telemetry statement before setting power to zero.
                    if ((LIFT_MEDIUM - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_MEDIUM + TOLERANCE)) {
                        telemetry.addData("Lift Low Status", "You've arrived at your MEDIUM destination");
                        lift.setPower(0);
                    }


                    // Sets and hold lift to High Junction height
                    else if (gamepad1.dpad_up) {
                        lift.setTargetPosition(LIFT_HIGH);
                        lift.setPower(0.5);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while (lift.isBusy()) {
                            // Send telemetry info to dashboard
                            telemetry.addData("Status", "Running lift to HIGH");
                            telemetry.addData("Lift Power", lift.getPower());

                            telemetry.update();
                        }

                        // Test the telemetry statement before setting power to zero.
                        if ((LIFT_HIGH - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + TOLERANCE)) {
                            telemetry.addData("Lift Low Status", "You've arrived at your HIGH destination");
                            lift.setPower(0);
                        }


                        // Sets and hold lift to zero position for cone grab-intake
                        else if (gamepad1.dpad_down) {
                            lift.setTargetPosition(0);
                            lift.setPower(1);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            // Feedback for what the motor is doing
                            while (lift.isBusy()) {
                                // Send telemetry info to dashboard
                                telemetry.addData("Status", "Running lift to ZERO");
                                telemetry.addData("Lift Power", lift.getPower());
                                telemetry.update();
                            }
                            // Test the telemetry statement before setting power to zero.
                            if ((-TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (TOLERANCE)) {
                                telemetry.addData("Lift Low Status", "You've arrived at the final destination");
                                lift.setPower(0);
                            }
                        } // End of lift commands

            /* Unused buttons at the moment
            gamepad2.left_bumper // Set lift to ground junction height
            gamepad2.dpad_down // Set lift to on-the-ground height
            gamepad2.dpad_right // Set lift top low height
            gamepad2.dpad_left // Set lift to medium height
            gamepad2.dpad_up // Set lift to high junction height
            */

                        // FIXME ?
                        if (isStopRequested()) return;

                        // Data to send to Driver Station
                        telemetry.addData("Status", "Run Time: " + runtime.toString());
                        telemetry.addData("Motors", "Front L (%.2f), Front R (%.2f)", frontLeftPower, frontRightPower);
                        telemetry.addData("Motors", "Back L (%.2f), Back R (%.2f)", backLeftPower, backRightPower);
                        telemetry.addData("Lift Position", liftPos);
                        telemetry.addData("Lift Power", lift.getPower());

                        // TODO Distance Sensor data
                        // generic DistanceSensor methods.

            /*
            telemetry.addData("deviceName", sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            */


                        // Color sensor
                        telemetry.addData("Red", color.red());
                        telemetry.addData("Green", color.green());
                        telemetry.addData("Blue", color.blue());

                        telemetry.update();
                    }
                }
            }
        }
    }
} // End of Class