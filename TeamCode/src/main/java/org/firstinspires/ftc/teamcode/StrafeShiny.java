package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


public class StrafeShiny extends LinearOpMode{

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    String direction = "right";

    @Override
    public void runOpMode() throws InterruptedException {

    }

    // Adding in perhaps to prevent the method from taking off?


    public StrafeShiny(String direction, double desiredEncoder, double Power) {

        // frontLeftMotor.setPower(0.0);
        // backRightMotor.setPower(0.0);
        // frontRightMotor.setPower(0.0);
        // backLeftMotor.setPower(0.0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction.equals("left")) {
            frontLeftMotor.setPower(-Power);
            backRightMotor.setPower(-Power);

            frontRightMotor.setPower(Power);
            backLeftMotor.setPower(Power);

            while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * -desiredEncoder))
                ;
            while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * -desiredEncoder))
                ;
            while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * desiredEncoder))
                ;
            while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED * desiredEncoder))
                ;
        }
        if (direction.equals("right")) {
            frontLeftMotor.setPower(Power);
            backRightMotor.setPower(Power);

            frontRightMotor.setPower(-Power); // (-) strafe right
            backLeftMotor.setPower(-Power); // (-) strafe right

            while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder)) ;
            while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder));

            while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * -desiredEncoder)); // (-) strafe right
            while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED  * -desiredEncoder)) ; // (-) strafe right
        }

        } // end of strafe

}
