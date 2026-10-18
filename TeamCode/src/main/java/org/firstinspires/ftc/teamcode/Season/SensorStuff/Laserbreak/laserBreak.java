package org.firstinspires.ftc.teamcode.Season.SensorStuff.Laserbreak;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

public class laserBreak implements Mechanism {

    private boolean laserDetect = true;
    private DigitalChannel laserInput;
    public boolean laserInputState = false;

    public laserBreak() {}

    public void init(HardwareMap hardwareMap) {
        try {
            laserInput = hardwareMap.get(DigitalChannel.class, "laserbreak");
            laserInput.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            laserInput = null;
        }
    }

    @Override
    public void periodic() {
        Mechanism.super.periodic();

        if (laserDetect && laserInput != null) {
            laserInputState = laserInput.getState();
        }
    }
}
