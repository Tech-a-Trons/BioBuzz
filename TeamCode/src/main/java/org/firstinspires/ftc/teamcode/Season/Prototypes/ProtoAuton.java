package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;

public class ProtoAuton extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)
    ProtoBot robot;
    private ProtoAuton(ProtoBot robot) { super(robot); }

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1Start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(10.1564, 44.9306, 180);
    private final Pose path1Control1 = poseFactory.of(41.7153, 46.4628, 0);
    private final Pose path1Control2 = poseFactory.of(30.3857, 47.8247, 0);
    private final Pose point2 = poseFactory.of(56.1985, 123.5295, 90);
    private final Pose point2Control1 = poseFactory.of(22.7903, 104.5877, 0);
    private final Pose point3 = poseFactory.of(44.871, 132.223, 90);
    private final Pose point4 = poseFactory.of(54.1255, 26.087, 270);
    private final Pose point4Control1 = poseFactory.of(9.4032, 45.2721, 0);


    //Sets all the paths that the robot needs to go to
    private Path path1() {
        return curve(path1Start, path1Control1, path1Control2, path1).linear(path1Start, path1);
    }

    private Path path2() {
        return curve(path1, point2Control1, point2).linear(path1, point2);
    }

    private Path path3() {
        return line(point2, point3).constant(point3);
    }

    private Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }

    //For Pedro 3, the delays must be put as commands and defined earlier than the sequencer
    //See: https://pedropathing.com/docs/ivy/utilities-and-decorators#wait
    Command shootpause = waitMs(750); // waits 500ms
    Command intpause = waitMs(500); // waits 500ms

    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                shootpause,
                follow(follower, path1()),
                robot.intake.run(),
                intpause,
                follow(follower, path2()),
                shootpause,
                follow(follower, path3()),
                intpause,
                follow(follower, path4()),
                shootpause
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
