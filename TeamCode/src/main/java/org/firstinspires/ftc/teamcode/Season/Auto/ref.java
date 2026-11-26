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
public class ref extends NextOpMode {
    ProtoBot robot;
    public ref(ProtoBot robot) { super(robot); this.robot = robot; }

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(62.0969, 132.8062, 270);
    private final Pose path1 = poseFactory.of(14.2284, 113.5342, 270);
    private final Pose path1Control1 = poseFactory.of(64.5087, 108.362, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }

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
                follow(follower, path1())
//                robot.intakeTransfer.feedSlow(),
//                follow(follower, path3().with(Constants.foresightConfig.maxPathSpeed.at(.55))),
//                quick(),
//                follow(follower, special().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
//                follow(follower, special2().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
//                gardenw(),
//                robot.intakeTransfer.stop(),
//                follow(follower, path4()),
//                parallel(robot.shooter.setSpinning(true),
//                        robot.hoodGate.openGate()),
//                quick(),
//                robot.intakeTransfer.feed(),
//                shooter(),
//                parallel(robot.shooter.setSpinning(false),
//                        robot.hoodGate.closeGate(),
//                        robot.intakeTransfer.stop()
//                ),
//                follow(follower, path5()),
//                parallel(
//                        robot.flowerRamp.down(),
//                        robot.intakeTransfer.feedSlow()
//                ),
//                follow(follower, path6().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
//                flowerw(),
//                follow(follower, jig1().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
//                follow(follower, jig2().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
//                follow(follower, path7().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
//                parallel(robot.intakeTransfer.stop(),
//                        robot.flowerRamp.up()),
//                follow(follower, path8()),
//                follow(follower,path9()),
//                parallel(robot.shooter.setSpinning(true),
//                        robot.hoodGate.openGate()),
//                quick(),
//                robot.intakeTransfer.feed(),
//                shooter(),
//                parallel(robot.shooter.setSpinning(false),
//                        robot.hoodGate.closeGate(),
//                        robot.intakeTransfer.stop()
//                ),
//                follow(follower,path10()),
//                follow(follower,path11())
//                //,
////                follow(follower, path9()),
////                parallel(
////                        robot.flowerRamp.down(),
////                        robot.intakeTransfer.feedSlow()
////                ),
////                follow(follower, path10().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
////                flowerw(),
////                follow(follower, jig3().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
////                follow(follower, jig4().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
////                follow(follower, path11().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
////                parallel(robot.intakeTransfer.stop(),
////                        robot.flowerRamp.up()),
////                follow(follower, path12()),
////                parallel(robot.shooter.setSpinning(true),
////                        robot.hoodGate.openGate()),
////                quick(),
////                robot.intakeTransfer.feed(),
////                shooter(),
////                parallel(robot.shooter.setSpinning(false),
////                        robot.hoodGate.closeGate(),
////                        robot.intakeTransfer.stop()
////                )
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
