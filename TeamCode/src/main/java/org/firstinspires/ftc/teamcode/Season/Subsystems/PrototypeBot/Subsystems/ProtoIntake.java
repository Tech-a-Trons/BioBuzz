package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class ProtoIntake implements Mechanism {
    NextMotor motor = new NextMotor("intake");

    public Command run() { return infinite(() -> motor.setThrottle(1)); }
    public Command mid() {return infinite(() -> motor.setThrottle(0.5));}
    public Command slow() {return infinite(() -> motor.setThrottle(0.25));}

    public Command stop() { return instant(() -> motor.setThrottle(0.0)); }
    public Command reverse() { return instant(() -> motor.setThrottle(-1)); }
    //add more cmds if needed, also make sure that all new commands with a power above 0 are infinite
    //but 0 power is instant
}
