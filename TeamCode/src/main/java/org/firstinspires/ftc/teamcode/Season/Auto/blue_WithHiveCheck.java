package org.firstinspires.ftc.teamcode.Season.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class blue_WithHiveCheck extends NextOpMode {
    ProtoBot robot;

    public blue_WithHiveCheck(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose shootUp = poseFactory.of(84, 134, 90);
    private final Pose garden = poseFactory.of(137.6937, 134.8121, 0);
    private final Pose shootDown = poseFactory.of(84, 6.9455, 90);

    // Autonomous routine
    public Command autoRoutine() {
        if (!robot.tip.checkBlue()) {
            return sequential(
                    follow(follower, Intake1()),
                    follow(follower, Garden_to_Shoot()),
                    follow(follower, SideSwitchUpDown())
            );
        } else {
            return sequential(
                    follow(follower, Intake1()),
                    follow(follower, Garden_to_Shoot())
            );
        }
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(shootUp);
        follower.update();
        robot.tip.init(hardwareMap);
    }

    @Override
    public void start() {
        Telemetry.log("Status", "Start");
        schedule(autoRoutine());
    }

    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Tag ID", robot.tip.tagId);
        telemetry.addData("Blue Front", robot.tip.blueFront);
        telemetry.update();
    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }

    public Path Intake1() {
        return line(shootUp, garden).linear(shootUp, garden);
    }

    public Path Garden_to_Shoot() {
        return line(garden, shootUp).linear(garden, shootUp);
    }

    public Path SideSwitchUpDown() {
        return line(shootUp, shootDown).linear(shootUp, shootDown);
    }

    public Path SideSwitchDownUp() {
        return line(shootDown, shootUp).linear(shootDown, shootUp);
    }
}
