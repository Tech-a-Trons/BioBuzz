package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoOuttake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.hiveTip;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class ProtoBot implements NextRobot {

    public ProtoIntake intake = new ProtoIntake();
    public Drivetrain drive = new Drivetrain();
    public ProtoOuttake outtake = new ProtoOuttake();
    public hiveTip tip = new hiveTip();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, drive, outtake, tip);
    }
}
