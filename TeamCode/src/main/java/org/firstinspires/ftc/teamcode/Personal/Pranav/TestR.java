package org.firstinspires.ftc.teamcode.Personal.Pranav;

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

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.OdoPID;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class TestR extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)
    ProtoBot robot;
    OdoPID pid;

    public TestR(ProtoBot robot) { super(robot); this.robot = robot;}

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.7758, 8, 270);
    private final Pose path1 = poseFactory.of(55.7758, 23.8906, 270);
    private final Pose point2 = poseFactory.of(40.0281, 40.4019, 180);
    private final Pose point2Control1 = poseFactory.of(41.6965, 25.5388, 0);
    private final Pose point3 = poseFactory.of(18.3986, 40.2845, 180);
    private final Pose point4 = poseFactory.of(28.3673, 40.6224, 180);
    private final Pose point5 = poseFactory.of(57.267, 110.6918, 90);
    private final Pose point5Control1 = poseFactory.of(25.454, 117.1038, 0);

    public Path path1() {
        return line(start, path1).constant(path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).constant(point3);
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    //Command spause = waitMs(1000); // waits 500ms
    Command intpause = waitMs(1000); // waits 500ms
    Command warmup = waitMs(2000);
    Command spause = waitMs(750); // waits 500ms

    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                pid.run(),
                warmup,
                robot.intake.run(),
                robot.transfer.run(),
                spause,
                follow(follower, path1()),
                robot.intake.run(),
                robot.transfer.slow(),
                follow(follower, path2()),
                follow(follower, path3()),
                intpause,
                robot.intake.stop(),
                follow(follower, path4()),
                pid.run(),
                follow(follower,path5()),
                //warmup,
                robot.intake.run(),
                robot.transfer.run(),
                spause
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
