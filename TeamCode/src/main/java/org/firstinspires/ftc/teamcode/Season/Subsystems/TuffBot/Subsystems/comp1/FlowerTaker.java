package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
public class FlowerTaker implements Mechanism {
    Servo flower1,flower2;

    public void init(HardwareMap hardwareMap) {
        flower1 = hardwareMap.servo.get("flower1");
        flower2 = hardwareMap.servo.get("flower2");
    }

    public Command f1start() { return instant(() -> flower1.setPosition(0.7)); }
    public Command f2start() { return instant(() -> flower2.setPosition(0.05)); }

    public Command f1down() { return instant(() -> flower1.setPosition(0.05)); }
    public Command f2down() { return instant(() -> flower2.setPosition(0.7)); }
}
