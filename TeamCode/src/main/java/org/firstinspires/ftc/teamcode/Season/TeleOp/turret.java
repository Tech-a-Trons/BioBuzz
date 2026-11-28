package org.firstinspires.ftc.teamcode.Season.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Robot.ProtoBot;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop
public class turret extends AllianceTeleop {
    public turret(ProtoBot robot) { super(robot, "RED"); }
}