package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous

public class ColorTest extends LinearOpMode {
    // Define a variable for our color sensor

    ColorSensor sensorColor; // name in Robot Config on Driver's Station

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue", sensorColor.blue());
            telemetry.update();
        }
    }
}