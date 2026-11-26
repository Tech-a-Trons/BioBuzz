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
    private final Pose special = poseFactory.of(16.72, 5.09, 185);
    private final Pose jig = poseFactory.of(43.31, 119.61, 90);
    private final Pose jig2 = poseFactory.of(43.31, 123.21, 90);
    private final Pose jig3 = poseFactory.of(-0.25, 58.2, 180);
    private final Pose jig4 = poseFactory.of(-1, 58.33, 180);


    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.6829, 23.7929, 90);
    private final Pose point2Start = poseFactory.of(55.6829, 23.7929, 90);
    private final Pose point2 = poseFactory.of(56.1752, 7.882, 180);
    private final Pose point3 = poseFactory.of(8.6416, 4.5901, 185);
    private final Pose point4Start = poseFactory.of(8.6416, 4.5901, 180);
    private final Pose point4 = poseFactory.of(48.7, 129, 270);
    private final Pose point4Control1 = poseFactory.of(21.9819, 122.0783, 0);
    private final Pose point4Control2 = poseFactory.of(59.0194, 112.8552, 0);
    private final Pose point5Start = poseFactory.of(48.7, 129, 270);
    private final Pose point5 = poseFactory.of(43.068, 112.3914, 90);
    private final Pose point6 = poseFactory.of(43.2944, 129.4933, 90);
    private final Pose point7 = poseFactory.of(43.1226, 106.4734, 90);
    private final Pose point8Start = poseFactory.of(43.1226, 112.4734, 90);
    private final Pose point8 = poseFactory.of(54.938, 112.6384, 260);
    private final Pose point9 = poseFactory.of(54.9036, 130, 270);

    private final Pose point10 = poseFactory.of(54.8896, 112.6602, 270);
    private final Pose point11 = poseFactory.of(11.5767, 112.6181, 270);
//    private final Pose point9Start = poseFactory.of(48.7, 116.89, 267);
//    private final Pose point9 = poseFactory.of(15.9734, 57.4107, 180);
//    private final Pose point9Control1 = poseFactory.of(13.8648, 114.3748, 0);
//    private final Pose point10 = poseFactory.of(-2.15, 58.4249, 180);
//    private final Pose point11 = poseFactory.of(8.6444, 57.426, 180);
//    private final Pose point12Start = poseFactory.of(8.6444, 57.426, 180);
//    private final Pose point12 = poseFactory.of(46.7744, 23.7963, 90);
//    private final Pose point12Control1 = poseFactory.of(25.9654, 28.7487, 0);

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
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point8, point9).constant(point9);
    }
    public Path path10() {
        return Paths.line(point9, point10).constant(point10);
    }

    public Path path11() {
        return Paths.line(point10, point11).constant(point11);
    }


//    public Path path9() {
//        return Paths.curve(point9Start, point9Control1, point9).linear(point9Start, point9);
//    }
//
//    public Path path10() {
//        return Paths.line(point9, point10).constant(point10);
//    }
//
//
//    public Path path12() {
//        return Paths.curve(point12Start, point12Control1, point12).linear(point12Start, point12);
//    }

    public Path path7() {
        return Paths.line(jig2, point7).constant(point7);
    }
    public Path special() {
        return Paths.line(point3, special).constant(special);
    }

    public Path special2() {
        return Paths.line(special, point3).constant(point3);
    }
    public Path jig1() {
        return Paths.line(point6, jig).constant(jig);
    }

    public Path jig2() {
        return Paths.line(jig, jig2).constant(jig2);
    }
//    public Path jig3() {
//        return Paths.line(point10, jig3).constant(jig3);
//    }
    public Path jig4() {
        return Paths.line(jig3, jig4).constant(jig4);
    }
//    public Path path11() {
//        return Paths.line(jig4, point11).constant(point11);
//    }

    Command gardenw() {return waitMs(500);}
    Command flowerw() {return waitMs(700);}
    Command shooter() {return waitMs(2500);}
    Command quick() {return waitMs(1500);}


    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                robot.flowerRamp.up(),
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.flowerRamp.up(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),
                //quick(),
//                parallel(
//                        robot.hoodGate.closeGate(),
//                        robot.flowerRamp.up()
//                ),
                //Call mechanisms here
                //follow(follower, path2()),
                follow(follower, path1()),
                robot.intakeTransfer.feedSlow(),
                follow(follower, path3().with(Constants.foresightConfig.maxPathSpeed.at(.55))),
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
                follow(follower, jig1().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                follow(follower, jig2().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
                follow(follower, path7().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
                parallel(robot.intakeTransfer.stop(),
                        robot.flowerRamp.up()),
                follow(follower, path8()),
                follow(follower,path9()),
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),
                follow(follower,path10()),
                follow(follower,path11())
                //,
//                follow(follower, path9()),
//                parallel(
//                        robot.flowerRamp.down(),
//                        robot.intakeTransfer.feedSlow()
//                ),
//                follow(follower, path10().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
//                flowerw(),
//                follow(follower, jig3().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
//                follow(follower, jig4().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
//                follow(follower, path11().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
//                parallel(robot.intakeTransfer.stop(),
//                        robot.flowerRamp.up()),
//                follow(follower, path12()),
//                parallel(robot.shooter.setSpinning(true),
//                        robot.hoodGate.openGate()),
//                quick(),
//                robot.intakeTransfer.feed(),
//                shooter(),
//                parallel(robot.shooter.setSpinning(false),
//                        robot.hoodGate.closeGate(),
//                        robot.intakeTransfer.stop()
//                )
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
        robot.hoodGate.initPositions();
        robot.flowerRamp.up();
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
