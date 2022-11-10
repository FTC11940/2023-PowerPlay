package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.LIFT_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;
import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Lifting extends LinearOpMode {

    Hardwell   robot       = new Hardwell(this);

    private Servo grabby;
    private DcMotor lift;


    public void highJunction() {
        lift.setTargetPosition(LIFT_HIGH);
        lift.setPower(1.0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Test the telemetry statement before setting power to zero.
        if ((LIFT_HIGH - TOLERANCE) < lift.getCurrentPosition() && lift.getCurrentPosition() < (LIFT_HIGH + TOLERANCE)) {
            telemetry.addData("Lift Low Status", "You've arrived at your HIGH destination");
            // lift.setPower(0);
            sleep(300); // this may vary too much? Or figure out when this needs to be time triggered
            grabby.setPosition(OPEN);
            sleep(300);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
