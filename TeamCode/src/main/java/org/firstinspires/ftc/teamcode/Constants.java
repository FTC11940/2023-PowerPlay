/*
 * Constants used across the our FTC code base
 * Naming conventions https://docs.oracle.com/javase/tutorial/java/nutsandbolts/variables.html
 */

package org.firstinspires.ftc.teamcode;

public class Constants {

    // Preset lift heights using encoder ticks
    public static final int LIFT_FLOOR = 0; // 1" Ground junction height
    public static final int LIFT_GROUND = 100; // 1" Ground junction height
    public static final int LIFT_TOP_STACK = 300; // for use in Auton
    public static final int LIFT_THREE_STACK = 232; // for use in Auton
    public static final int LIFT_TWO_STACK = 166; // for use in Auton
    public static final int LIFT_LOW = 1200; // 13.5" Low junction height
    public static final int LIFT_MEDIUM = 2000; // 23.5" Medium junction height
    public static final int LIFT_HIGH = 3000; // 33.5" High junction height
    public static final int TOLERANCE = 10;

    // Same as above for lift, just different names
    public static final int lift_floor = 0;
    public static final int lift_ground = 100; // 1" Ground junction height
    public static final int lift_low = 1200; // 13.5" Low junction height
    public static final int lift_mid = 2000; // 23.5" Medium junction height
    public static final int lift_high = 3000; // 33.5" High junction height
    public static final int diplomat = 10;

    //constant lift power
    public static final double lift_power = 1;
    //constant lift bumper speed
    public static final int bee_speed = 50;
    //bumper-gamepad1-strafe encoder value
    public static final int hornet_speed = 25000;

    // Constants related to the claw
    public static final double OPEN = 0.33;
    public static final double CLOSED = 0.0;

    // what position grabby, or the claw, considers completely open
    public static final double grabby_open = 0.2;
    // what position grabby, or the claw, considers completely closed
    public static final double grabby_closed = 0.0;

    // Autonomous constants
    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED = 0.45;
    public static final double HEADING_THRESHOLD = 1.0;

    public static final double P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_GAIN           = 0.00;     // Larger is more responsive, but also less stable

    // Sets the servo YSNP a.k.a. "You Shall Not Pass"
    public static final double SHUT = 0.1;
    public static final double PASS = 0.5;


    // Vuforia constants
    public static final String VUFORIA_KEY =
            "AQlm0VH/////AAABmecSieTQsU26rgeA9+8pNJ0JiCO/pfP8X82+Kv4Czexw7qCupZvMaBQROIVF3rj9aoXmjtRDTA1mHJVdaV6hWpfm0jMW8qnLFUmIJQwHIYZ9aM/sQR71rjIx9L5REWzQ9LTYIwLGaW3gwBbaOXpOZ/yi2mt4Q/RIes7idEhwoRkFQh0H+zHxO0iqZdbbUibMlTMif/0g/FqqjBY/abecyaNvfCErZX6jVLlMXPk4cyEeCO6i57ufYKhrOMAHa2BOEgbjvTx52vUeZHh5nORqSa/mJy7ZzE2mrSG4SJ7KEZm1C7Lbnzo6WtMj+F6e1gqISfrynyNJoMrlq9uUHzm8gEbcebYncHx869F+ykNDQTZ7";
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */

    public static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    // Use this if adding custom signal images
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    // Conversions from encoder ticks to inches with the lift.
    // Proof of concept more than actual usage for this season

    // ORIGINAL used with Failsafe 11-13-2022 and conceptually the lift
    /*
    public static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    public static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    public static final double PULLEY_DIAMETER_INCHES = 2.0 ; // For figuring circumference
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);
    */

    // Failsafe
    public static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    public static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    public static final double PULLEY_DIAMETER_INCHES = 2.0 ; // For figuring circumference
    public static final double FAILSAFE_COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);


    // Conversions from encoder ticks to inches with the Drivetrain.
    // public static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket - same as above
    public static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    public static final double WHEEL_DIAMETER_INCHES = 4.01575  ; // For figuring circumference // website 3.93701
    public static final double DRIVE_COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

}