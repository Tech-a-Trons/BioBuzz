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

@NextAutonomous (name = "Blue 1st Tip Partner 1")
public class FirstTipPartner1Blue extends NextOpMode {
    ProtoBot robot;
    public FirstTipPartner1Blue(ProtoBot robot) { super(robot); this.robot = robot; }

    Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Field is 180° rotationally symmetric (not a left/right mirror).
    // (x, y, h) -> (W - x - X_OFF, W - y - Y_OFF, h + 180); same constants as SoloBlue.
    private static final double FIELD_SIZE = 144;
    private static final double X_OFF = 2;
    private static final double Y_OFF = 3;

    private Pose rotate(double x, double y, double headingDeg) {
        double h = ((headingDeg + 180) % 360 + 360) % 360;
        return poseFactory.of(FIELD_SIZE - x - X_OFF, FIELD_SIZE - y - Y_OFF, h);
    }

    // ---- Red coordinates, rotated 180° ----
    private final Pose special = rotate(16.72, 5.09, 185);

    private final Pose start = rotate(56, 8, 90);
    private final Pose path1 = rotate(55.68, 23.79, 90);
    private final Pose point2Start = rotate(55.68, 23.79, 90);
    private final Pose point2 = rotate(56.18, 7.88, 180);
    private final Pose point3 = rotate(8.64, 4.59, 185);
    private final Pose point4Start = rotate(8.64, 4.59, 180);
    private final Pose point4 = rotate(56.15, 127.62, 270);
    private final Pose point4Control1 = rotate(67.43, 2.81, 0);
    private final Pose point5 = rotate(6.43, 118.3423, 270);

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

    Command gardenw() { return waitMs(500); }
    Command flowerw() { return waitMs(450); }
    Command shooter() { return waitMs(1000); }
    Command quick()   { return waitMs(600); }

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
                follow(follower, path5())
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