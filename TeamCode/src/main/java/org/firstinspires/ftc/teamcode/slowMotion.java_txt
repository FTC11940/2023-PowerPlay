package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.MecanumDriveDEV.front;

public class slowMotion {
        double ySlow = -gamepad1.left_stick_y; // Uses the left thumbstick for left and right robot movement
        double xSlow = gamepad1.left_stick_x; //*1.1 to counteract imperfect strafing
        double rotSlow = gamepad1.right_stick_x; // Uses the right thumbstick to rotate robot movement
        int slowFactor = 4;

        double frontLeftPowerSlow = (ySlow / slowFactor + xSlow / slowFactor + rotSlow / slowFactor);
        double backLeftPowerSlow = (ySlow / slowFactor - xSlow / slowFactor + rotSlow / slowFactor);
        double frontRightPowerSlow = (ySlow / slowFactor - xSlow / slowFactor - rotSlow / slowFactor);
        double backRightPowerSlow = (ySlow / slowFactor + xSlow / slowFactor - rotSlow / slowFactor);

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPowerSlow);
        backLeftMotor.setPower(backLeftPowerSlow);
        frontRightMotor.setPower(frontRightPowerSlow);
        backRightMotor.setPower(backRightPowerSlow);


        // sleep(100); // Works but adds a stutter


} // end of slowMo method
