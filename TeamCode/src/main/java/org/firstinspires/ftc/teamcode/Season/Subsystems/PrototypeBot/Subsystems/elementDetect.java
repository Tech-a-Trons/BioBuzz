package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import org.firstinspires.ftc.teamcode.Season.SensorStuff.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.SensorStuff.Laserbreak.laserBreak;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import dev.nextftc.robot.Mechanism;

public class elementDetect implements Mechanism {

    private final ColorSensor colorSensor;
    private final laserBreak laserBreak;

    public elementDetect(ColorSensor colorSensor, laserBreak laserBreak) {
        this.colorSensor = colorSensor;
        this.laserBreak = laserBreak;
    }

    public boolean elementDetect = true;
    public final List<Integer> rampElements = new ArrayList<>(Arrays.asList(null, null, null, null));


    // 1 = pollen
    // 2 = Red_Nectar
    // 3 = Blue_Nectar

    public void rampClear() {
        for (int i = 0; i < rampElements.size(); i++) {
            rampElements.set(i, null);
        }
        colorSensor.pollen = 0;
        colorSensor.red_nectar = 0;
        colorSensor.blue_nectar = 0;
    }

    public void periodic() {
        Mechanism.super.periodic();
        if (colorSensor.colorDetect) {
            colorSensor.getColor();
        }

        if (laserBreak.laserDetect && laserBreak.laserInput != null) {
            laserBreak.laserInputState = laserBreak.laserInput.getState();
        }
        if (elementDetect) {
            if (!laserBreak.laserInputState) {
                colorSensor.color_counted = false;
            }
            if (laserBreak.laserInputState) {
                int elementType = colorSensor.IncountBalls();
                if (elementType != 0) {
                    for (int i = 0; i < rampElements.size(); i++) {
                        if (rampElements.get(i) == null) {
                            rampElements.set(i, elementType);
                            break;
                        }
                    }
                }
            }
        }
    }
}
