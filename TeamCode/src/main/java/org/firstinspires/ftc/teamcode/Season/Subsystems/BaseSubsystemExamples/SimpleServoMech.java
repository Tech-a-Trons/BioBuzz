package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class SimpleServoMech implements Mechanism {

    //Init the servo
    NextServo servo = new NextServo("servo name");

    //Defines everything u want the servo to do
    public Command open() { return instant(() -> servo.setPosition(0.2)); }
    public Command close() { return instant(() -> servo.setPosition(0.8)); }
    ///change positions based on application!
    //add more cmds if needed, also make sure that all new commands with a power above 0 are infinite
    //but 0 power is instant
}