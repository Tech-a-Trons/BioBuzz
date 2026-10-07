package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.Transfer;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class ProtoBot implements NextRobot {

    //Calls all of the mechanisms
//    public Simpleflywheelmech flywheel = new Simpleflywheelmech();
public ProtoIntake intake = new ProtoIntake();
public Drivetrain drive = new Drivetrain();
public Shooter shoot = new Shooter();
public Transfer tranboy = new Transfer();
//    public SimpleServoMech servo = new SimpleServoMech();

    //Essentially defines each mechanism as part of the robot, can be call in other programs
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(shoot,intake,drive,tranboy);
    }
}
