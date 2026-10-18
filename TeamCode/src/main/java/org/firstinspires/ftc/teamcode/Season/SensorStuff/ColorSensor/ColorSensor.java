package org.firstinspires.ftc.teamcode.Season.SensorStuff.ColorSensor;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class ColorSensor implements Mechanism {
    private RevColorSensorV3 colorsensor;
    private Servo rgbindicator;

    private boolean initialized = false;
    private final boolean colorDetect = true;

    private float chue, csat, cval;
    private float red, green, blue, alpha;

    public float blue_nectar = 0;
    public float red_nectar = 0;
    public float pollen = 0;

    public boolean color_counted = false;

    public ColorSensor() {}

    // ================= INIT =================

    public void init(HardwareMap hardwareMap) {
        try {
            colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");
            try {
                rgbindicator = hardwareMap.get(Servo.class, "rgbindicator");
            } catch (Exception ignored) {}
            initialized = true;
        } catch (Exception e) {
            initialized = false;
        }
        blue_nectar = 0;
        red_nectar = 0;
        pollen = 0;
    }

    // ================= COLOR DETECTION =================

    public String getColor() {
        if (!initialized || colorsensor == null) return null;

        red = colorsensor.red();
        green = colorsensor.green();
        blue = colorsensor.blue();
        alpha = colorsensor.alpha();

        float total = red + green + blue;
        if (total <= 0) return null;

        double rNorm = red / total;
        double gNorm = green / total;
        double bNorm = blue / total;

        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0];
        csat = hsv[1];
        cval = hsv[2];

        return null;
    }

    // ================= BALL COUNTING =================

    public int IncountBalls() {
        if (!initialized || colorsensor == null || color_counted) return 0;

        if (chue > 190) {
            blue_nectar += 1;
            color_counted = true;
            return 3; // Blue_Nectar
        } else if (chue < 130 && chue > 90) {
            pollen += 1;
            color_counted = true;
            return 1; // pollen
        } else if (chue < 85) {
            red_nectar += 1;
            color_counted = true;
            return 2; // Red_Nectar
        }

        return 0;
    }

    // ================= PERIODIC =================

    @Override
    public void periodic() {
        if (colorDetect) {
            getColor();
        }
    }

    // ================= GETTERS =================

    public float getHue() { return chue; }
    public float getSat() { return csat; }
    public float getVal() { return cval; }
    public float getRed() { return red; }
    public float getGreen() { return green; }
    public float getBlue() { return blue; }
    public float getAlpha() { return alpha; }
}
