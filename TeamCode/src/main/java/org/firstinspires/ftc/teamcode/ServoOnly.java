package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/****************************
    runOpMode(): Code inside this method will run exactly once after you press the INIT button. This is where you should put all code for the OpMode.
    waitForStart(): This method pauses the Op-Mode until you press the START button on the driver station.
    isStarted(): returns true if the START button has been pressed, otherwise it returns false.
    isStopRequested(): returns true if the STOP button has been pressed, otherwise it returns false.
    idle(): calls Thread.yield, allowing other threads at the same priority level to run.
    opModeIsActive(): returns isStarted() && !isStopRequested() and calls idle().
    opModeInInit(): returns !isStarted() && !isStopRequested() and does not call idle().
 *****************************/

@TeleOp(name = "Servo Only", group="Linear OpMode")
public class ServoOnly extends LinearOpMode {

    // private ElapsedTime runtime = new ElapsedTime(); //Added from BasicOpLinear
    Servo grabby;
    double grabberPosOpen = 0.0; // TODO test the servo ; 0.5 is open, 0.0 is closed
    double grabberPosClosed = 0.5; // TODO test the servo ; 0.5 is open, 0.0 is closed
    // final double GRABBER_SPEED = 0.01; // TODO test the servo

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); //Added from BasicOpLinear
        telemetry.update(); //Added from BasicOpLinear

        grabby = hardwareMap.servo.get("grabby");
        grabby.setPosition(grabberPosClosed);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // TODO Test the servo code block
            // uses the buttons a and y to open and close
            if(gamepad1.y) {grabby.setPosition(grabberPosOpen);} // Sets claw to open position
            if(gamepad1.a) {grabby.setPosition(grabberPosClosed);} // Sets claw to closed position

            /* Telemetry Data

             */
            // telemetry.addData("claw", grabby.setPosition();); // gives the servo values to the phone for viewing
        }
    }
} // End of Class