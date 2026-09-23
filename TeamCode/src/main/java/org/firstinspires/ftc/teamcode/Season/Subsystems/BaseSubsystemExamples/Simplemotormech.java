package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Simplemotormech implements Mechanism {

    //Init the motor
    NextMotor motor = new NextMotor("motorname");

    //Defines every single action that is needed

    public Command run() { return infinite(() -> motor.setThrottle(0.75)); }
    public Command burst() {return infinite(() -> motor.setThrottle(1));}
    public Command slow() {return infinite(() -> motor.setThrottle(0.25));}

    public Command stop() { return instant(() -> motor.setThrottle(0.0)); }
    public Command vomit() {return infinite(() -> motor.setThrottle(-0.5));}
    //add more cmds if needed, also make sure that all new commands with a power above 0 are infinite
    //but 0 power is instant
}
