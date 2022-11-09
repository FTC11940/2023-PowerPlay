/*
 * Constants used across the our FTC code base
 * Naming conventions https://docs.oracle.com/javase/tutorial/java/nutsandbolts/variables.html
 */

package org.firstinspires.ftc.teamcode;

public class Constants {


    public static final int LIFT_GROUND = 100; // 1" Ground junction height
    public static final int LIFT_LOW = 1200; // 13.5" Low junction height
    public static final int LIFT_MEDIUM = 2000; // 23.5" Medium junction height
    public static final int LIFT_HIGH = 2800; // 33.5" High junction height
    public static final int TOLERANCE = 10;

    // Same as above for lift, just different variable names
    public static final int lift_floor = 0;
    public static final int lift_ground = 100; // 1" Ground junction height
    public static final int lift_low = 1200; // 13.5" Low junction height
    public static final int lift_mid = 2000; // 23.5" Medium junction height
    public static final int lift_high = 2800; // 33.5" High junction height
    public static final int diplomat = 10;


    public static final double OPEN = 0.5;
    public static final double CLOSED = 0.0;
    public static final double GRABBY_OPEN = 0.5;
    public static final double GRABBY_CLOSED = 0.0;

    // Autonomous constants
    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED = 0.45;
    public static final double HEADING_THRESHOLD = 1.0;

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


}