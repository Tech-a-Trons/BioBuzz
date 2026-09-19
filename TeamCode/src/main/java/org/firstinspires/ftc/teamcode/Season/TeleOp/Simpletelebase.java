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

        //Calls the classes and method defined in the NextRobot class

        if (gamepad1.left_bumper) {
            robot.flywheel.run();
        }
        if (gamepad1.right_bumper) {
            robot.flywheel.stop();
        }
        if (gamepad1.a) {
            robot.motor.burst();
        }
        if (gamepad1.x) {
            robot.motor.slow();
        }
        if (gamepad1.b) {
            robot.motor.stop();
        }
        if (gamepad1.y) {
            robot.motor.run();
        }
        if (gamepad1.dpad_down) {
            robot.servo.open();
        }
        if (gamepad1.dpad_right) {
            robot.servo.close();
        }
    }

    //Runs once at stop
    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
