package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.CLOSED;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;
import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift extends LinearOpMode {

    private Servo grabby;
    private DcMotor lift;

    @Override
    public void runOpMode() {
        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(CLOSED); // Needs to be closed at start of Auton
        lift = hardwareMap.get(DcMotor.class,"lift");
    }

    // Adding in perhaps to prevent the method from taking off?
    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    /* TODO See if needed
    while (void opModeIsActive()) {

    }
     */

    public Lift(int height, double Power) {
        lift.setTargetPosition(height);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((height - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (height + TOLERANCE)) {
            telemetry.addData("Lift Low Status", "You've arrived at your destination");
            // lift.setPower(0);

            sleep(300); // this may vary too much? Or figure out when this needs to be time triggered

            grabby.setPosition(OPEN);

            sleep(300);
        }
        // lift.setPower(0.0);
    };

}
