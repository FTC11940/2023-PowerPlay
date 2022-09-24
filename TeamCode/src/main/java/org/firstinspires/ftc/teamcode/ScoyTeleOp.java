package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ScoyTeleOp", group="Linear OpMode")
public class ScoyTeleOp extends LinearOpMode {
    // Makes a timer variable
    private ElapsedTime runtime = new ElapsedTime();
    // Define motors
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    private DcMotor clawServo;

    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();
    }
}