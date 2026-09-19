package org.firstinspires.ftc.teamcode.Season.TeleOp;


import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplemotormech;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoIntake;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;


@NextTeleop
public class protoTeleop extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)


    ProtoBot robot;
    public protoTeleop(ProtoBot robot) { super(robot); }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
    }

    @Override
    public void start() {
        Telemetry.log("Status", "Start");
        robot.drive.startDrive(gamepad1);
    }

    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");
        if (gamepad1.a){
            robot.intake.run();
        }
        if (gamepad1.x){
            robot.intake.stop();
        }
        if(gamepad1.b){
            robot.intake.reverse();
        }

    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
