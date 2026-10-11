package org.firstinspires.ftc.teamcode.Season.TeleOp;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

@NextTeleop
public class protoTeleop extends NextOpMode {

    ProtoBot robot;

    public protoTeleop(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }
    private double targetRpm = 1000;
    private static final double STEP = 100;
    private static final double MAX_RPM = 6000;
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
//robot.intake.run() robot.intake.stop(),
        gp1.leftBumper().onTrue(parallel( robot.shoot.stop(),robot.tranboy.stop()));
        gp1.rightBumper().onTrue(parallel(robot.tranboy.run()));
        gp1.rightTrigger().isBetween(0.05,1).whileTrue(robot.shoot.run(400));
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