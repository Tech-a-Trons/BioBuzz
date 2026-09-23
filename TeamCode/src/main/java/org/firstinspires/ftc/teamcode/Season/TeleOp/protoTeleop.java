package org.firstinspires.ftc.teamcode.Season.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop
public class protoTeleop extends NextOpMode {

    ProtoBot robot;

    public protoTeleop(ProtoBot robot) {
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
        robot.drive.startDrive(gamepad1);

        gp1.a().onTrue(robot.intake.run());
        gp1.x().onTrue(robot.intake.stop());
        gp1.b().onTrue(robot.intake.reverse());
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