package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import com.pedropathing.follower.Follower;

@NextTeleop
public class Steering extends NextOpMode {

    ProtoBot robot;
    Follower follower = Constants.create(hardwareMap);

    public Steering(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }

    double tx = 0;
    private Limelight3A limelight3A;
    boolean tracking = false;
    boolean valid = false;

    @Override
    public void disabledPeriodic() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight3A");
        limelight3A.pipelineSwitch(0);
    }
    @Override
    public void start() {
        limelight3A.start();

        if (gamepad1.a) {
            tracking = true;

        }
        if (gamepad1.b) {
            tracking = false;
            follower.manual(
                    0,
                    0,
                    0
            );        }
    }
    @Override
    public void periodic() {
        LLResult llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target X offest", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());


            tx = llResult.getTx();
        }

        valid = (llResult != null && llResult.isValid());
        if (tracking && !valid) {
            follower.manual(
                    0,
                    0,
                    3
            );
        } else if (tracking && valid && !(tx < 0.1 && -0.1 < tx)) {
            follower.manual(
                    0,
                    0,
                    3 * tx
            );

        } else if (tracking && valid && (tx < 0.1 && -0.1 < tx)) {
            follower.manual(
                    5,
                    0,
                    0
            );
            robot.intake.run();

        } else {
            follower.manual(
                    0,
                    0,
                    0
            );
            robot.intake.stop();

        }

    }
}