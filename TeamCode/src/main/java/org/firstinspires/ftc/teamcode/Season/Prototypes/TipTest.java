package org.firstinspires.ftc.teamcode.Season.Prototypes;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop
public class TipTest extends NextOpMode {
    ProtoBot robot;

    public TipTest(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "TipTest Init");
    }

    @Override
    public void start() {
        Telemetry.log("Status", "TipTest Started");
        robot.tip.init(hardwareMap);
    }

    @Override
    public void periodic() {
        Telemetry.log("Status", "TipTest Running");
        Telemetry.log("Tag ID", String.valueOf(robot.tip.tagId));
        Telemetry.log("Red Front", String.valueOf(robot.tip.redFront));
        Telemetry.log("Blue Front", String.valueOf(robot.tip.blueFront));

        if (gamepad1.a) {
            robot.tip.limelightOn = true;
        } else if (gamepad1.b) {
            robot.tip.limelightOn = false;
        }
    }

    @Override
    public void end() {
        Telemetry.log("Status", "TipTest Ended");
    }
}
