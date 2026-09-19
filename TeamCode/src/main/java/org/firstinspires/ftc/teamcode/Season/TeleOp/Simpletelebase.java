package org.firstinspires.ftc.teamcode.Season.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplemotormech;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;


@NextTeleop
public class Simpletelebase extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)

    Simplenextrobot robot;

    public Simpletelebase(Simplenextrobot robot) { super(robot); }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
    }

    @Override
    public void start() {
        Telemetry.log("Status", "Start");
    }

    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");

        if (gamepad1.a) {
            robot.getMechanisms();
        }
    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
