package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_SPEED;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FRC 3603 Launch High

public class Strafe extends LinearOpMode{

    Servo grabby;
    DcMotor lift;

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void strafeLeft (double maxPower, double desiredEncoder) {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftMotor.setPower(-maxPower); // (-) strafe left
        backRightMotor.setPower(-maxPower); // (-) strafe left

        frontRightMotor.setPower(maxPower);
        backLeftMotor.setPower(maxPower);

        while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * -desiredEncoder)) ; // (-) strafe left
        while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * -desiredEncoder)); // (-) strafe left

        while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * desiredEncoder)) ;
        while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED  * desiredEncoder)) ;
    } // end strafe left

    public void strafeRight (double maxPower, double desiredEncoder) {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(maxPower);
        backRightMotor.setPower(maxPower);

        frontRightMotor.setPower(-maxPower); // (-) strafe right
        backLeftMotor.setPower(-maxPower); // (-) strafe right

        while (opModeIsActive() && (frontLeftMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder)) ;
        while (opModeIsActive() && (backRightMotor.getCurrentPosition() > DRIVE_SPEED * desiredEncoder));

        while (opModeIsActive() && (backLeftMotor.getCurrentPosition() < DRIVE_SPEED * -desiredEncoder)); // (-) strafe right
        while (opModeIsActive() && (frontRightMotor.getCurrentPosition() < DRIVE_SPEED  * -desiredEncoder)) ; // (-) strafe right
    } // end of strafe right
}
