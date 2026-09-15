package org.firstinspires.ftc.teamcode.Personal.Pranav;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {
    NextMotor motor = new NextMotor("transfer");

    public Command run() { return infinite(() -> motor.setThrottle(0.75)); }
    public Command burst() {return infinite(() -> motor.setThrottle(1));}
    public Command slow() {return infinite(() -> motor.setThrottle(0.25));}

    public Command stop() { return instant(() -> motor.setThrottle(0.0)); }
}
