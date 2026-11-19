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

@NextAutonomous (name = "Red 1st Tip Partner 2")
public class FirstTipPartner2 extends NextOpMode {
    ProtoBot robot;
    public FirstTipPartner2(ProtoBot robot) { super(robot); this.robot = robot; }

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose special = poseFactory.of(16.72, 5.09, 185);
    private final Pose jig3 = poseFactory.of(12.5, 47.47, 180);
    private final Pose jig4 = poseFactory.of(12.15, 47.47, 180);

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.68, 23.79, 90);
    private final Pose point2Start = poseFactory.of(55.68, 23.79, 90);
    private final Pose point2 = poseFactory.of(56.18, 7.88, 180);
    private final Pose point3 = poseFactory.of(8.64, 4.59, 185);
    private final Pose point4Start = poseFactory.of(8.64, 4.59, 180);
    private final Pose point4 = poseFactory.of(56.1539, 127.6215, 270);
    private final Pose point4Control1 = poseFactory.of(67.4345, 2.8106, 0);
    private final Pose point5Start = poseFactory.of(56.1539, 127.6215, 270);
    private final Pose point5 = poseFactory.of(21.6882, 49.5597, 180);
    private final Pose point5Control1 = poseFactory.of(16.0891, 113.4008, 0);
    private final Pose point6 = poseFactory.of(10.75, 47.47, 180);
    private final Pose point7 = poseFactory.of(16.7811, 49.2889, 180);
    private final Pose point8Start = poseFactory.of(16.7811, 49.2889, 180);
    private final Pose point8 = poseFactory.of(46.77, 13.8, 90);
    private final Pose point8Control1 = poseFactory.of(25.97, 28.75, 0);
    private final Pose point9 = poseFactory.of(12.0496, 90.8176, 90);
    private final Pose point9Control1 = poseFactory.of(8.0459, 21.2574, 0);

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
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9).constant(point9);
    }
    public Path special() {
        return Paths.line(point3, special).constant(special);
    }

    public Path special2() {
        return Paths.line(special, point3).constant(point3);
    }
    public Path jig3() {
        return Paths.line(point6, jig3).constant(jig3);
    }
    public Path jig4() {
        return Paths.line(jig3, jig4).constant(jig4);
    }
    public Path path7() {
        return Paths.line(jig4, point7).constant(point7);
    }

    Command gardenw() {return waitMs(500);}
    Command flowerw() {return waitMs(200);}
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
                follow(follower, path5()),
                parallel(
                        robot.flowerRamp.down(),
                        robot.intakeTransfer.feedSlow()
                ),
                follow(follower, path6().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                flowerw(),
                follow(follower, jig3().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                follow(follower, jig4().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
                follow(follower, path7().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
                parallel(robot.intakeTransfer.stop(),
                        robot.flowerRamp.up()),
                follow(follower, path8()),
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),
                follow(follower,path9())
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
