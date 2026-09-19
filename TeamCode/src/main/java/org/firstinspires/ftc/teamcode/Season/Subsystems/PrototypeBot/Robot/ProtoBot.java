package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoIntake;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class ProtoBot implements NextRobot {

    //Calls all of the mechanisms
//    public Simpleflywheelmech flywheel = new Simpleflywheelmech();
public ProtoIntake motor = new ProtoIntake();
//    public SimpleServoMech servo = new SimpleServoMech();

    //Essentially defines each mechanism as part of the robot, can be call in other programs
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(motor);
    }
}
