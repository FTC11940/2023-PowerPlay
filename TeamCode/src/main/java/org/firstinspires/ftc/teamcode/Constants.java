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
        public static final int LIFT_LOW = 1200; // 13.5" Low junction height
        public static final int LIFT_MEDIUM = 2000; // 23.5" Medium junction height
        public static final int LIFT_HIGH = 2800; // 33.5" High junction height

        //
        public static final double     DRIVE_SPEED             = 0.6;   // Max driving speed for better distance accuracy.
        public static final double     TURN_SPEED              = 0.45;   // Max Turn speed to limit turn rate
        public static final double     HEADING_THRESHOLD       = 1.0 ;  // How close must the heading get to the target before moving to next step.

        public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
        public static final double     P_DRIVE_GAIN           = 0.00;     // Larger is more responsive, but also less stable


        // SetPosition() values for Open and Close on the claw servo
        public static final double OPEN = 0.5;
        public static final double CLOSED = 0.0;
        public static final int TOLERANCE = 10; // Lift encoder tolerance
        // top of cone stack is 918 rotations

    }