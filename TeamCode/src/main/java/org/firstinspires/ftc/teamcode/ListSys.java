package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.LIFT_FLOOR;
import static org.firstinspires.ftc.teamcode.Constants.OPEN;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class ListSys {

    private LinearOpMode opForce = null;

    // Create motors
    private DcMotor lift = null; // creates the lift motor

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ListSys(LinearOpMode opmode) {
        opForce = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        lift = opForce.hardwareMap.get(DcMotor.class, "lift");

        lift.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setPower(0);

        lift.setTargetPosition(LIFT_FLOOR);


    } // end of init

} // End class