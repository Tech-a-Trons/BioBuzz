package org.firstinspires.ftc.teamcode.Personal.Pranav;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor motor = new NextMotor("intake");

    public Command run() { return infinite(() -> motor.setThrottle(1.0)); }
    public Command stop() { return instant(() -> motor.setThrottle(0.0)); }
}
