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

    // SetTargetPosition(0) is the game field zero
    public static final int LIFT_GROUND = 100; // 1" Ground junction height
    public static final int LIFT_LOW = 2000; // 12" Low junction height
    public static final int LIFT_MEDIUM = 4000; // 18" Medium junction height
    public static final int LIFT_HIGH = 6000; // 32" High junction height

    // SetPosition() values for Open and Close on the claw servo
    public static final double OPEN = 0.5;
    public static final double CLOSED = 0.0;

}
