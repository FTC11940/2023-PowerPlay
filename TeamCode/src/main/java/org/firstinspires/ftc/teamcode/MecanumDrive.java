package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Drive", group="Linear OpMode")
public abstract class MecanumDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(); //Added from BasicOpLinear

    // renamed to match Driver Hub
    // assigned 0 in Driver Hub
    private DcMotor frontRightMotor = null;
    // assigned 1 in Driver Hub
    private DcMotor frontLeftMotor = null;
    // assigned 2 in Driver Hub
    private DcMotor backRightMotor = null;
    // assigned 3 in Driver Hub
    private DcMotor backLeftMotor = null;

    //this function is started when this Op Mode is selected from the drives station

    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); //Added from BasicOpLinear
        telemetry.update(); //Added from BasicOpLinear
        float Vertical;
        float Horizontal;
        float Pivot;

        frontRightMotor = hardwareMap.get(DcMotor.class,"Front Right");
        frontLeftMotor = hardwareMap.get(DcMotor.class,"Front Left");
        backRightMotor = hardwareMap.get(DcMotor.class,"Back Right");
        backLeftMotor = hardwareMap.get(DcMotor.class,"Back Left");

        // Both right side motors should be going in one direction, and both left side motors going in the opposite direction
        // FIXME Naming conventions from Hardware Map to TeleOp
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO Test motor direction
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);// TODO Test motor direction
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO Test motor direction
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // TODO Test motor direction

        /*
         * From the Hardware Map
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
         */


        waitForStart();
        //added
        runtime.reset();

        if(opModeIsActive()){

            /* This may not be optimal. Consider using
            // Uses the left thumbstick for forward & backwards robot movement
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            */

            Vertical = -gamepad1.left_stick_y;
            // Uses the left thumbstick for left and right robot movement
            Horizontal = gamepad1.left_stick_x;
            // Uses the right thumbstick to rotate robot movement
            Pivot = gamepad1.right_stick_x;

            // TODO Test these directional settings
            frontRightMotor.setPower(-Pivot + (Vertical - Horizontal));
            frontLeftMotor.setPower(Pivot + Vertical + Horizontal);
            backLeftMotor.setPower(Pivot + (Vertical - Horizontal));
            backRightMotor.setPower(-Pivot + Vertical + Horizontal);

            // Send calculated power to wheels // Added from BasicOpLinear
            // leftDrive.setPower(leftPower);
            // rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); //Added from BasicOpLinear
            telemetry.update();

        }
    }
}