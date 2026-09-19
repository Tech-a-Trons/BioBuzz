package org.firstinspires.ftc.teamcode.Season.Auto;

import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;

public class Simpleautobase extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)
    Simplenextrobot robot;
    public Simpleautobase(Simplenextrobot robot) { super(robot); }

    //Runs periodically in init
    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
    }

    //Runs once at start
    @Override
    public void start() {
        Telemetry.log("Status", "Start");
    }

    //Runs periodically through the game
    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");
    }

    //Runs once at stop
    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
