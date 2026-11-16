package org.firstinspires.ftc.teamcode.Season.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class SoloRed extends NextOpMode {
    ProtoBot robot;
    public SoloRed(ProtoBot robot) { super(robot); this.robot = robot; }

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.6829, 23.7929, 90);
    private final Pose point2Start = poseFactory.of(55.6829, 23.7929, 90);
    private final Pose point2 = poseFactory.of(55.787, 18.1694, 180);
    private final Pose point3 = poseFactory.of(15.8234, 18.2742, 180);
    private final Pose point4Start = poseFactory.of(15.8234, 18.2742, 180);
    private final Pose point4 = poseFactory.of(57.2236, 112.8884, 270);
    private final Pose point4Control1 = poseFactory.of(16.8382, 111.888, 0);
    private final Pose point5Start = poseFactory.of(57.2236, 112.8884, 270);
    private final Pose point5 = poseFactory.of(46.6589, 112.3914, 90);
    private final Pose point6 = poseFactory.of(46.8853, 124.7758, 90);
    private final Pose point7 = poseFactory.of(46.9076, 112.0852, 90);
    private final Pose point8Start = poseFactory.of(46.9076, 112.0852, 90);
    private final Pose point8 = poseFactory.of(57.0758, 112.8084, 270);
    private final Pose point9Start = poseFactory.of(57.0758, 112.8084, 270);
    private final Pose point9 = poseFactory.of(24.9021, 47.1233, 180);
    private final Pose point9Control1 = poseFactory.of(26.0932, 112.3367, 0);
    private final Pose point10 = poseFactory.of(17.4433, 47.167, 180);
    private final Pose point11 = poseFactory.of(25.046, 47.1386, 180);
    private final Pose point12Start = poseFactory.of(25.046, 47.1386, 180);
    private final Pose point12 = poseFactory.of(55.7031, 23.6992, 90);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point9Start, point9Control1, point9).linear(point9Start, point9);
    }

    public Path path10() {
        return Paths.line(point9, point10).constant(point10);
    }

    public Path path11() {
        return Paths.line(point10, point11).constant(point11);
    }

    public Path path12() {
        return Paths.line(point12Start, point12).linear(point12Start, point12);
    }

    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                //Call mechanisms here
                //follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9()),
                follow(follower, path10()),
                follow(follower, path11()),
                follow(follower, path12())
        );
    }

    //Runs periodically in init
    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");

        //Sets up the scheduler and calls in the Constants
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    //Runs once at start
    @Override
    public void start() {
        Telemetry.log("Status", "Start");

        //Makes the auto start when play button is pressed
        schedule(autoRoutine());
    }

    //Runs periodically through the game
    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");

        //Starts follower and auto path
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    //Runs once at stop
    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
