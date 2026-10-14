package org.firstinspires.ftc.teamcode.Season.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.OdoPID;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public class OdoTele extends NextOpMode {

    ProtoBot robot;

    OdoPID odoshoot;

    public OdoTele(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");

        odoshoot.init(hardwareMap);
        odoshoot.reset();

        odoshoot.alliance = "RED";
        odoshoot.Localization();
    }

    @Override
    public void start() {
        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        Telemetry.log("Status", "Start");
        robot.drive.startDrive(gamepad1);
        robot.tip.init(hardwareMap);

        gp1.a().onTrue(robot.intake.run());
        gp1.x().onTrue(robot.intake.stop());
        gp1.b().onTrue(robot.intake.reverse());

        if (gamepad1.right_bumper) {
            odoshoot.OdoRun();
        }

        if (gamepad1.right_trigger_pressed) {
            odoshoot.reset();
        }

        if (gamepad1.dpad_right) {
            odoshoot.Localization();
            odoshoot.reset();
        }

        if (gamepad1.left_bumper) {
            odoshoot.run();
        }

        if (gamepad1.dpad_up) {
            odoshoot.alliance = "BLUE";
        }

        if (gamepad1.dpad_up) {
            odoshoot.alliance = "RED";
        }

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