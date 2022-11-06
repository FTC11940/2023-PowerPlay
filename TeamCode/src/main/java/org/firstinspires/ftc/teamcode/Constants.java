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
    public static final int TOLERANCE = 10;

    // SetPosition() values for Open and Close on the claw servo
    public static final double OPEN = 0.5;
    public static final double CLOSED = 0.0;

    //
    public static final int lift_floor = 0;
    public static final int lift_ground = 100; // 1" Ground junction height
    public static final int lift_low = 1200; // 13.5" Low junction height
    public static final int lift_mid = 2000; // 23.5" Medium junction height
    public static final int lift_high = 2800; // 33.5" High junction height
    public static final int diplomat = 10;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AQlm0VH/////AAABmecSieTQsU26rgeA9+8pNJ0JiCO/pfP8X82+Kv4Czexw7qCupZvMaBQROIVF3rj9aoXmjtRDTA1mHJVdaV6hWpfm0jMW8qnLFUmIJQwHIYZ9aM/sQR71rjIx9L5REWzQ9LTYIwLGaW3gwBbaOXpOZ/yi2mt4Q/RIes7idEhwoRkFQh0H+zHxO0iqZdbbUibMlTMif/0g/FqqjBY/abecyaNvfCErZX6jVLlMXPk4cyEeCO6i57ufYKhrOMAHa2BOEgbjvTx52vUeZHh5nORqSa/mJy7ZzE2mrSG4SJ7KEZm1C7Lbnzo6WtMj+F6e1gqISfrynyNJoMrlq9uUHzm8gEbcebYncHx869F+ykNDQTZ7";
}
