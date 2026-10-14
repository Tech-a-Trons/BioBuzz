package org.firstinspires.ftc.teamcode.Season.SensorStuff.Laserbreak;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.robot.Mechanism;

public class laserBreak implements Mechanism {

    boolean laserDetect = true;

    private DigitalChannel laserInput;

    public void Init() {
        // Get the digital sensor from the hardware map
        laserInput = hardwareMap.get(DigitalChannel.class, "laserbreak");

        // Set the channel as an input
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the driver to press PLAY

        // Loop while the OpMode is active
            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = laserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = stateHigh;

            // Display detection state
            if (detected) {
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
            }

            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.update();
        }
    @Override
    public void periodic() {
        Mechanism.super.periodic();

        if (laserDetect = true) {
            boolean laserInputState = laserInput.getState();
        }
    }
}