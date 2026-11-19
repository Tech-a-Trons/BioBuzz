package org.firstinspires.ftc.teamcode.Season.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
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
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Robot.ProtoBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous (name = "Red 1st Tip Partner 1")
public class FirstTipPartner1 extends NextOpMode {
    ProtoBot robot;
    public FirstTipPartner1(ProtoBot robot) { super(robot); this.robot = robot; }

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose special = poseFactory.of(16.72, 5.09, 185);

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.68, 23.79, 90);
    private final Pose point2Start = poseFactory.of(55.68, 23.79, 90);
    private final Pose point2 = poseFactory.of(56.18, 7.88, 180);
    private final Pose point3 = poseFactory.of(8.64, 4.59, 185);
    private final Pose point4Start = poseFactory.of(8.64, 4.59, 180);
    private final Pose point4 = poseFactory.of(56.15, 127.62, 270);
    private final Pose point4Control1 = poseFactory.of(67.43, 2.81, 0);
    private final Pose point5 = poseFactory.of(6.43, 118.3423, 270);

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
        return Paths.line(point4, point5).constant(point5);
    }
    public Path special() {
        return Paths.line(point3, special).constant(special);
    }

    public Path special2() {
        return Paths.line(special, point3).constant(point3);
    }

    Command gardenw() {return waitMs(500);}
    Command flowerw() {return waitMs(450);}
    Command shooter() {return waitMs(1000);}
    Command quick() {return waitMs(600);}


    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                robot.flowerRamp.up(),
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),
                follow(follower, path1()),
                robot.intakeTransfer.feedSlow(),
                follow(follower, path3().with(Constants.foresightConfig.maxPathSpeed.at(.35))),
                quick(),
                follow(follower, special().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
                follow(follower, special2().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
                gardenw(),
                robot.intakeTransfer.stop(),
                follow(follower, path4()),
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),
                follow(follower, path5())
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

        robot.init(hardwareMap,follower,"RED");
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
