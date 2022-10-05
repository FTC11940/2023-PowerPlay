package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Only", group="Linear OpMode")
public class ServoOnly extends LinearOpMode {

    // private ElapsedTime runtime = new ElapsedTime(); //Added from BasicOpLinear
    Servo grabby;
    double grabberPosOpen = 0.0; // 0.5 is open, 0.0 is closed
    double grabberPosClosed = 0.5; // 0.5 is open, 0.0 is closed
    // final double GRABBER_SPEED = 0.01; // Not needed at this point

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(grabberPosClosed);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Uses the buttons a and y to open and close
            if(gamepad1.y) {grabby.setPosition(grabberPosOpen);} // Sets claw to open position
            if(gamepad1.a) {grabby.setPosition(grabberPosClosed);} // Sets claw to closed position

            /* Telemetry Data
             */
            // FIXME This is not correct and needs fixing. Idea is to give the servo values to the phone for viewing
            // telemetry.addData("claw", grabby.setPosition(););
        }
    }
} // End of Class