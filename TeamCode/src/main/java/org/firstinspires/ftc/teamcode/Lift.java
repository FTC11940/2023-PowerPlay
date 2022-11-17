package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;

public class Lift extends LinearOpMode {

    // Needed to initialize the variables?!
    private Servo grabby = null;
    private DcMotor lift = null;

    @Override
    public void runOpMode() {
        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(CLOSED); // Needs to be closed at start of Auton
        lift = hardwareMap.get(DcMotor.class, "lift");
    }

    /*
    while (void opModeIsActive()) {

    }
     */

    int height = 0;
    double power = 0.0;

    public Lift(int height, double power) {

        lift.setTargetPosition(height);
        lift.setPower(OPEN);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((height - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (height + TOLERANCE)) {
            telemetry.addData("Lift Low Status", "You've arrived at your destination");
            // lift.setPower(0);

        }
        // lift.setPower(0.0);
    }

    ;

}
