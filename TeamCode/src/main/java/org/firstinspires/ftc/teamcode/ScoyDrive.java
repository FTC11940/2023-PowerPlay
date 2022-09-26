package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/****************************
    runOpMode(): Code inside this method will run exactly once after you press the INIT button. This is where you should put all code for the OpMode.
    waitForStart(): This method pauses the Op-Mode until you press the START button on the driver station.
    isStarted(): returns true if the START button has been pressed, otherwise it returns false.
    isStopRequested(): returns true if the STOP button has been pressed, otherwise it returns false.
    idle(): calls Thread.yield, allowing other threads at the same priority level to run.
    opModeIsActive(): returns isStarted() && !isStopRequested() and calls idle().
    opModeInInit(): returns !isStarted() && !isStopRequested() and does not call idle().
 *****************************/

@TeleOp(name = "Scoy Drive", group="Linear OpMode")
public class ScoyDrive extends LinearOpMode {

    // private ElapsedTime runtime = new ElapsedTime(); //Added from BasicOpLinear

    // renamed to match Driver Hub
    /* Located in the Hardware file
    */
    private DcMotor frontRightMotor = null; // assigned 0 in Driver Hub
    private DcMotor frontLeftMotor = null; // assigned 1 in Driver Hub
    private DcMotor backRightMotor = null; // assigned 2 in Driver Hub
    private DcMotor backLeftMotor = null; // assigned 3 in Driver Hub

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); //Added from BasicOpLinear
        telemetry.update(); //Added from BasicOpLinear


        frontRightMotor = hardwareMap.get(DcMotor.class,"Front Right");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front Left");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back Right");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back Left");

        // Both right side motors should be going in one direction, and both left side motors going in the opposite direction
        /* This appears to be set already in the hardware map
        */

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);// TODO Test motor direction
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO Test motor direction
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // TODO Test motor direction
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // TODO Test motor direction


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /* This may not be optimal. Consider using
            // Uses the left thumbstick for forward & backwards robot movement
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */


            double vertical = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
            double horizontal = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
            double pivot = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement

            // Variables for wheel motor power and inputs
            double frontLeftPower = (vertical + horizontal + pivot);
            double backLeftPower = (vertical - horizontal + pivot);
            double frontRightPower = (vertical - horizontal - pivot);
            double backRightPower = (vertical + horizontal - pivot);

            // TODO Test these directional settings
            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            // leftDrive.setPower(leftPower);
            // rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            /* Taking out for simplicity
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); //Added from BasicOpLinear
            telemetry.update();
             */

        }
    }
} // End of Class