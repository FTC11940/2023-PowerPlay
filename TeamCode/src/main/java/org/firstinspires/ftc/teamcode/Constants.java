/* 
Constants file for FTC code base
*/
package org.firstinspires.ftc.teamcode;

public class Constants {

    /* SetTargetPosition() for the lift preset encoder values. 
    Values to be determined experimental and set here
    ADD import static org.firstinspires.ftc.teamcode.Constants.*;
    to each file reference constants
    */
    //the set positions of each height the lift needs to be at
    public static final int lift_floor = 0;
    public static final int lift_ground = 100;
    public static final int lift_low = 1200;
    public static final int lift_mid = 2000;
    public static final int lift_high = 2800;
    // makes sure that the teleop is accurate, originally had more use but no real reason to scrap it now
    public static final int diplomat = 10;

    // what position grabby, or the claw, considers completely open
    public static final double grabby_open = 0.2;

    //constant lift power
    public static final double lift_power = 0.5;
    //constant lift bumper speed
    public static final int bee_speed = 50;
    //bumper-gamepad1-strafe encoder value
    public static final int hornet_speed = 5000;

}
