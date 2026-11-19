package org.firstinspires.ftc.teamcode.Season.Auto;

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

@NextAutonomous
public class SoloBlue extends NextOpMode {
    ProtoBot robot;
    public SoloBlue(ProtoBot robot) { super(robot); this.robot = robot; }

    Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // The field is 180° rotationally symmetric, not a left/right mirror.
    // Rotate each Red pose about the field center: (x, y, h) -> (W - x - X_OFF, W - y - Y_OFF, h + 180)
    // Offsets are fitted so Red's start (56, 8) lands on Blue's start (86, 133).
    private static final double FIELD_SIZE = 144;
    private static final double X_OFF = 2;
    private static final double Y_OFF = 3;

    private Pose rotate(double x, double y, double headingDeg) {
        double h = ((headingDeg + 180) % 360 + 360) % 360;
        return poseFactory.of(FIELD_SIZE - x - X_OFF, FIELD_SIZE - y - Y_OFF, h);
    }

    // ---- Red coordinates, rotated 180° ----
    private final Pose special = rotate(16.72, 5.09, 185);
    private final Pose jig  = rotate(43.31, 119.61, 90);
    private final Pose jig2 = rotate(43.31, 123.21, 90);
    private final Pose jig3 = rotate(-0.25, 58.2, 180);
    private final Pose jig4 = rotate(-1, 58.33, 180);

    private final Pose start = rotate(56, 8, 90);
    private final Pose path1 = rotate(55.6829, 23.7929, 90);
    private final Pose point2Start = rotate(55.6829, 23.7929, 90);
    private final Pose point2 = rotate(56.1752, 7.882, 180);
    private final Pose point3 = rotate(8.6416, 4.5901, 185);
    private final Pose point4Start = rotate(8.6416, 4.5901, 180);
    private final Pose point4 = rotate(57.2236, 112.8884, 270);
    private final Pose point4Control1 = rotate(21.9819, 122.0783, 0);
    private final Pose point5Start = rotate(57.2236, 112.8884, 270);
    private final Pose point5 = rotate(43.068, 112.3914, 90);
    private final Pose point6 = rotate(43.2944, 127.4933, 90);
    private final Pose point7 = rotate(43.1226, 112.4734, 90);
    private final Pose point8Start = rotate(43.1226, 112.4734, 90);
    private final Pose point8 = rotate(52.9996, 113.1966, 270);
    private final Pose point9Start = rotate(52.9996, 113.1966, 270);
    private final Pose point9 = rotate(15.9734, 57.4107, 180);
    private final Pose point9Control1 = rotate(13.8648, 114.3748, 0);
    private final Pose point10 = rotate(-2.15, 58.4249, 180);
    private final Pose point11 = rotate(8.6444, 57.426, 180);
    private final Pose point12Start = rotate(8.6444, 57.426, 180);
    private final Pose point12 = rotate(46.7744, 23.7963, 90);
    private final Pose point12Control1 = rotate(25.9654, 28.7487, 0);

    public Path path1()  { return Paths.line(start, path1).constant(path1); }
    public Path path2()  { return Paths.line(point2Start, point2).linear(point2Start, point2); }
    public Path path3()  { return Paths.line(point2, point3).constant(point3); }
    public Path path4()  { return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4); }
    public Path path5()  { return Paths.line(point5Start, point5).linear(point5Start, point5); }
    public Path path6()  { return Paths.line(point5, point6).constant(point6); }
    public Path path7()  { return Paths.line(jig2, point7).constant(point7); }
    public Path path8()  { return Paths.line(point8Start, point8).linear(point8Start, point8); }
    public Path path9()  { return Paths.curve(point9Start, point9Control1, point9).linear(point9Start, point9); }
    public Path path10() { return Paths.line(point9, point10).constant(point10); }
    public Path path11() { return Paths.line(jig4, point11).constant(point11); }
    public Path path12() { return Paths.curve(point12Start, point12Control1, point12).linear(point12Start, point12); }
    public Path special()  { return Paths.line(point3, special).constant(special); }
    public Path special2() { return Paths.line(special, point3).constant(point3); }
    public Path jig1() { return Paths.line(point6, jig).constant(jig); }
    public Path jig2() { return Paths.line(jig, jig2).constant(jig2); }
    public Path jig3() { return Paths.line(point10, jig3).constant(jig3); }
    public Path jig4() { return Paths.line(jig3, jig4).constant(jig4); }

    Command gardenw() { return waitMs(500); }
    Command flowerw() { return waitMs(450); }
    Command shooter() { return waitMs(1000); }
    Command quick()   { return waitMs(600); }

    // Same shoot sequence used repeatedly
    private Command shootSequence() {
        return sequential(
                parallel(robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()),
                quick(),
                robot.intakeTransfer.feed(),
                shooter(),
                parallel(robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop())
        );
    }

    private Command autoRoutine() {
        return sequential(
                robot.flowerRamp.up(),
                shootSequence(),
                follow(follower, path1()),
                robot.intakeTransfer.feedSlow(),
                follow(follower, path3().with(Constants.foresightConfig.maxPathSpeed.at(.35))),
                quick(),
                follow(follower, special().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
                follow(follower, special2().with(Constants.foresightConfig.maxPathSpeed.at(.5))),
                gardenw(),
                robot.intakeTransfer.stop(),
                follow(follower, path4()),
                shootSequence(),
                follow(follower, path5()),
                parallel(robot.flowerRamp.down(), robot.intakeTransfer.feedSlow()),
                follow(follower, path6().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                flowerw(),
                follow(follower, jig1().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                follow(follower, jig2().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
                follow(follower, path7().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
                parallel(robot.intakeTransfer.stop(), robot.flowerRamp.up()),
                follow(follower, path8()),
                shootSequence(),
                follow(follower, path9()),
                parallel(robot.flowerRamp.down(), robot.intakeTransfer.feedSlow()),
                follow(follower, path10().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                flowerw(),
                follow(follower, jig3().with(Constants.foresightConfig.maxPathSpeed.at(0.33))),
                follow(follower, jig4().with(Constants.foresightConfig.maxPathSpeed.at(0.35))),
                follow(follower, path11().with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
                parallel(robot.intakeTransfer.stop(), robot.flowerRamp.up()),
                follow(follower, path12()),
                shootSequence()
        );
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        robot.init(hardwareMap, follower, "BLUE");
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
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}