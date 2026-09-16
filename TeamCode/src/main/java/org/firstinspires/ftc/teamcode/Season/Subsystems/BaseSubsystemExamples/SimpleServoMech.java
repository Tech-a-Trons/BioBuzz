package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class SimpleServoMech implements Mechanism {
    NextServo servo = new NextServo("servo name");

    public Command open() { return instant(() -> servo.setPosition(0.2)); }
    public Command close() { return instant(() -> servo.setPosition(0.8)); }
    //change positions based on application!
}