package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive (Blocks to Java)")
public abstract class MecanumDrive extends LinearOpMode {
    private DcMotor Right_Front;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Left_Back;

    //this function is started when this Op Mode is selected from the drives station

    public void runOpMode() {
        float Vertical;
        float Horizontal;
        float Pivot;

        Right_Front = hardwareMap.get(DcMotor.class,"Right_Front");
        Right_Back = hardwareMap.get(DcMotor.class,"Right_Back");
        Left_Front = hardwareMap.get(DcMotor.class,"Left_Front");
        Left_Back = hardwareMap.get(DcMotor.class,"Left_Back");

        // Both right side motors should be going in one direction, and both left side motors going in the opposite direction
        // FIXME Naming conventions from Hardware Map to TeleOp
        Right_Front.setDirection(DcMotorSimple.Direction.FORWARD); // TODO Test motor direction
        Left_Front.setDirection(DcMotorSimple.Direction.REVERSE);// TODO Test motor direction
        Right_Back.setDirection(DcMotorSimple.Direction.FORWARD); // TODO Test motor direction
        Left_Back.setDirection(DcMotorSimple.Direction.REVERSE); // TODO Test motor direction

        /**
         * From the Hardware Map

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
         */


        waitForStart();

        if(opModeIsActive()){
            // Uses the left thumbstick for forward & backwards robot movement
            Vertical = -gamepad1.left_stick_y;
            // Uses the left thumbstick for left and right robot movement
            Horizontal = gamepad1.left_stick_x;
            // Uses the right thumbstick to rotate robot movement
            Pivot = gamepad1.right_stick_x;

            Right_Front.setPower(-Pivot + (Vertical - Horizontal));
            Right_Back.setPower(-Pivot + Vertical + Horizontal);
            Left_Front.setPower(Pivot + Vertical + Horizontal);
            Left_Back.setPower(Pivot + (Vertical - Horizontal));
        }
    }
}