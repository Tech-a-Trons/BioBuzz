package org.firstinspires.ftc.teamcode.Season.Prototypes;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public class MechTele extends NextOpMode {
    MechRobot robot;

    public MechTele(MechRobot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        Telemetry.log("Status", "Start");
        robot.drivetrain.startDrive(gamepad1);
    }

    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");
    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}

