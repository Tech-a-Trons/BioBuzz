package org.firstinspires.ftc.teamcode.Season.SensorStuff.Laserbreak;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

public class laserBreak implements Mechanism {

    public boolean laserDetect = true;
    public DigitalChannel laserInput;
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
    }
}
