
package org.firstinspires.ftc.teamcode.Season.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.lazy;
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
public class SoloRed extends NextOpMode {

    private final ProtoBot robot;
    private Follower follower;
    private boolean initialized = false;

    private String selectedBranch = "Not checked";

    public SoloRed(ProtoBot robot) {
        super(robot);
        this.robot = robot;
    }

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // POSES

    private final Pose special =
            poseFactory.of(16.72, 5.09, 185);

    private final Pose jig =
            poseFactory.of(43.31, 119.61, 90);

    private final Pose jig2 =
            poseFactory.of(43.31, 123.21, 90);

    private final Pose jig3 =
            poseFactory.of(-0.25, 58.2, 180);

    private final Pose jig4 =
            poseFactory.of(-1, 58.33, 180);

    private final Pose start =
            poseFactory.of(56, 8, 90);

    private final Pose path1 =
            poseFactory.of(55.6829, 23.7929, 90);

    private final Pose point2Start =
            poseFactory.of(55.6829, 23.7929, 90);

    private final Pose point2 =
            poseFactory.of(56.1752, 7.882, 180);

    private final Pose point3 =
            poseFactory.of(8.6416, 4.5901, 185);

    private final Pose point4Start =
            poseFactory.of(8.6416, 4.5901, 180);

    private final Pose point4 =
            poseFactory.of(57.2236, 112.8884, 270);

    private final Pose point4Control1 =
            poseFactory.of(21.9819, 122.0783, 0);

    private final Pose point5Start =
            poseFactory.of(57.2236, 112.8884, 270);

    private final Pose point5 =
            poseFactory.of(43.068, 112.3914, 90);

    private final Pose point6 =
            poseFactory.of(43.2944, 127.4933, 90);

    private final Pose point7 =
            poseFactory.of(43.1226, 112.4734, 90);

    private final Pose point8Start =
            poseFactory.of(43.1226, 112.4734, 90);

    private final Pose point8 =
            poseFactory.of(52.9996, 113.1966, 270);

    private final Pose point9Start =
            poseFactory.of(52.9996, 113.1966, 270);

    private final Pose point9 =
            poseFactory.of(15.9734, 57.4107, 180);

    private final Pose point9Control1 =
            poseFactory.of(13.8648, 114.3748, 0);

    private final Pose point10 =
            poseFactory.of(-2.15, 58.4249, 180);

    private final Pose point11 =
            poseFactory.of(8.6444, 57.426, 180);

    private final Pose point12Start =
            poseFactory.of(8.6444, 57.426, 180);

    private final Pose point12 =
            poseFactory.of(46.7744, 23.7963, 90);

    private final Pose point12Control1 =
            poseFactory.of(25.9654, 28.7487, 0);

    // PATHS

    public Path path1() {
        return Paths.line(start, path1)
                .constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2)
                .linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3)
                .constant(point3);
    }

    public Path path4() {
        return Paths.curve(
                point4Start,
                point4Control1,
                point4
        ).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5)
                .linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6)
                .constant(point6);
    }

    public Path path7() {
        return Paths.line(jig2, point7)
                .constant(point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8)
                .linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(
                point9Start,
                point9Control1,
                point9
        ).linear(point9Start, point9);
    }

    public Path path10() {
        return Paths.line(point9, point10)
                .constant(point10);
    }

    public Path path11() {
        return Paths.line(jig4, point11)
                .constant(point11);
    }

    public Path path12() {
        return Paths.curve(
                point12Start,
                point12Control1,
                point12
        ).linear(point12Start, point12);
    }

    public Path path13() {
        return Paths.line(point3, start)
                .constant(start);
    }

    public Path special() {
        return Paths.line(point3, special)
                .constant(special);
    }

    public Path special2() {
        return Paths.line(special, point3)
                .constant(point3);
    }

    public Path jig1() {
        return Paths.line(point6, jig)
                .constant(jig);
    }

    public Path jig2() {
        return Paths.line(jig, jig2)
                .constant(jig2);
    }

    public Path jig3() {
        return Paths.line(point10, jig3)
                .constant(jig3);
    }

    public Path jig4() {
        return Paths.line(jig3, jig4)
                .constant(jig4);
    }

    // WAIT COMMANDS

    private Command gardenw() {
        return waitMs(500);
    }

    private Command flowerw() {
        return waitMs(450);
    }

    private Command shooter() {
        return waitMs(1000);
    }

    private Command quick() {
        return waitMs(600);
    }

    // MAIN AUTONOMOUS

    private Command autoRoutine() {

        return sequential(

                robot.flowerRamp.up(),

                parallel(
                        robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()
                ),

                quick(),
                robot.intakeTransfer.feed(),
                shooter(),

                parallel(
                        robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),

                follow(follower, path1()),

                robot.intakeTransfer.feedSlow(),

                follow(
                        follower,
                        path3().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.35)
                        )
                ),

                quick(),

                follow(
                        follower,
                        special().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.5)
                        )
                ),

                follow(
                        follower,
                        special2().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.5)
                        )
                ),

                gardenw(),
                robot.intakeTransfer.stop(),

                follow(follower, path4()),

                // Allow 500 ms for camera and robot
                // to settle before choosing a path.
                waitMs(500),

                // Evaluate sensor at runtime.
                lazy(() -> {

                    // Force a Limelight refresh
                    // before making the decision.
                    robot.tip.periodic();

                    boolean red =
                            !robot.tip.checkRed();

                    selectedBranch =
                            red ? "RED" : "NO RED";

                    Telemetry.log(
                            "Selected Branch",
                            selectedBranch
                    );

                    if (red) {
                        return follow(follower, path1());
                    }

                    return noRedRoutine();
                })
        );
    }

    // NO RED ROUTINE

    private Command noRedRoutine() {

        return sequential(

                parallel(
                        robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()
                ),

                quick(),
                robot.intakeTransfer.feed(),
                shooter(),

                parallel(
                        robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),

                follow(follower, path5()),

                parallel(
                        robot.flowerRamp.down(),
                        robot.intakeTransfer.feedSlow()
                ),

                follow(
                        follower,
                        path6().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.33)
                        )
                ),

                flowerw(),

                follow(
                        follower,
                        jig1().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.33)
                        )
                ),

                follow(
                        follower,
                        jig2().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.35)
                        )
                ),

                follow(
                        follower,
                        path7().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.5)
                        )
                ),

                parallel(
                        robot.intakeTransfer.stop(),
                        robot.flowerRamp.up()
                ),

                follow(follower, path8()),

                parallel(
                        robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()
                ),

                quick(),
                robot.intakeTransfer.feed(),
                shooter(),

                parallel(
                        robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                ),

                follow(follower, path9()),

                parallel(
                        robot.flowerRamp.down(),
                        robot.intakeTransfer.feedSlow()
                ),

                follow(
                        follower,
                        path10().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.33)
                        )
                ),

                flowerw(),

                follow(
                        follower,
                        jig3().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.33)
                        )
                ),

                follow(
                        follower,
                        jig4().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.35)
                        )
                ),

                follow(
                        follower,
                        path11().with(
                                Constants.foresightConfig
                                        .maxPathSpeed.at(.5)
                        )
                ),

                parallel(
                        robot.intakeTransfer.stop(),
                        robot.flowerRamp.up()
                ),

                follow(follower, path12()),

                parallel(
                        robot.shooter.setSpinning(true),
                        robot.hoodGate.openGate()
                ),

                quick(),
                robot.intakeTransfer.feed(),
                shooter(),

                parallel(
                        robot.shooter.setSpinning(false),
                        robot.hoodGate.closeGate(),
                        robot.intakeTransfer.stop()
                )
        );
    }

    // INIT

    @Override
    public void disabledPeriodic() {

        if (!initialized) {

            Scheduler.reset();

            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            robot.init(
                    hardwareMap,
                    follower,
                    "RED"
            );

            // CRITICAL FIX:
            // Initialize Limelight explicitly,
            // just like blue_WithHiveCheck.
            robot.tip.init(hardwareMap);

            initialized = true;

            Telemetry.log(
                    "Status",
                    "SoloRed initialized"
            );
        }

        // Update sensor even before PLAY.
        robot.tip.periodic();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Tag ID", robot.tip.tagId);
        telemetry.addData(
                "Red Detected",
                robot.tip.checkRed()
        );
        telemetry.addData(
                "Blue Detected",
                robot.tip.checkBlue()
        );

        telemetry.addData(
                "Limelight Connected",
                robot.tip.limelight != null
                        && robot.tip.limelight.isConnected()
        );
    }

    // START

    @Override
    public void start() {

        if (!initialized || follower == null) {
            Telemetry.log(
                    "Error",
                    "Robot not initialized"
            );
            return;
        }

        Telemetry.log("Status", "Start");

        schedule(autoRoutine());
    }

    // AUTONOMOUS LOOP

    @Override
    public void periodic() {

        if (!initialized || follower == null) {
            return;
        }

        follower.update();

        // Keep tag detections updated while driving.
        robot.tip.periodic();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        // Sensor diagnostics
        telemetry.addData("Tag ID", robot.tip.tagId);
        telemetry.addData(
                "Red Detected",
                robot.tip.checkRed()
        );
        telemetry.addData(
                "Blue Detected",
                robot.tip.checkBlue()
        );

        telemetry.addData(
                "Limelight Connected",
                robot.tip.limelight != null
                        && robot.tip.limelight.isConnected()
        );

        telemetry.addData(
                "Branch",
                selectedBranch
        );

        // NextFTC handles the Ivy scheduler
        // and telemetry flushing.
    }

    // STOP

    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
