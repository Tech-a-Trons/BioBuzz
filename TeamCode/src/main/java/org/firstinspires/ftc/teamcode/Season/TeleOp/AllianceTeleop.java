package org.firstinspires.ftc.teamcode.Season.TeleOp;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Robot.ProtoBot;

import java.util.List;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.CommandGamepad;

public abstract class AllianceTeleop extends NextOpMode {

    private static final Pose FALLBACK_RED  = new Pose(70.75, 8.6, Math.toRadians(270));
    private static final Pose FALLBACK_BLUE = new Pose(141.5 - 70.75, 70.75, Math.PI - Math.toRadians(270));

    private static final Pose START_BLUE = new Pose(70.75, 8.6, Math.toRadians(270));
    private static final Pose START_RED  = new Pose(70.75, 8.6, Math.toRadians(270));

    private static final double DRIVER_FORWARD_RED  = Math.toRadians(270);
    private static final double DRIVER_FORWARD_BLUE = Math.toRadians(270);
    private static final long READY_HOLD_MS = 120;
    private static final long GATE_OPEN_DELAY_MS = 250;
    private static final long TELEMETRY_PERIOD_MS = 100;
    private static final boolean MANUAL_BULK_CACHING = true;

    private final ProtoBot robot;
    private final String alliance;
    private Follower follower;
    private List<LynxModule> hubs;

    private boolean feedPending = false;
    private boolean readyState = false;
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    boolean lastTrigger = false;
    int x = 0;

    protected AllianceTeleop(ProtoBot robot, String alliance) {
        super(robot);
        this.robot = robot;
        this.alliance = alliance;
    }

    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
    }

    @Override
    public void start() {
        Telemetry.log("Status", "Start");

        if (MANUAL_BULK_CACHING) {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.create(hardwareMap);
        Pose fallback = "BLUE".equals(alliance) ? FALLBACK_BLUE : FALLBACK_RED;
        follower.setPose(PoseStorage.pose != null ? PoseStorage.pose : fallback);

        robot.init(hardwareMap, follower, alliance);
        robot.tip.init(hardwareMap);

        robot.drive.setHeadingSource(() -> follower.pose().heading());
        robot.drive.setForwardHeading("BLUE".equals(alliance) ? DRIVER_FORWARD_BLUE : DRIVER_FORWARD_RED);
        robot.drive.startDrive(gamepad1);

        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        gp1.a().onTrue(robot.intakeTransfer.collect());
        gp1.a().onTrue(instant(() -> feedPending = false));
        gp1.rightBumper().onTrue(robot.intakeTransfer.feedSlow());
        gp1.rightBumper().onTrue(robot.hoodGate.closeGate());
        gp1.leftBumper().onTrue(robot.intakeTransfer.stop());
        gp1.leftBumper().onTrue(robot.hoodGate.closeGate());
        gp1.leftBumper().onTrue(instant(() -> feedPending = false));

        gp1.dpadUp().onTrue(robot.hoodGate.presetUp());
        gp1.dpadDown().onTrue(robot.hoodGate.presetDown());

        gp1.dpadRight().onTrue(instant(() ->
                follower.setPose("BLUE".equals(alliance) ? START_BLUE : START_RED)));

        // Set default red LED status at start
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        loopTimer.reset();
    }

    @Override
    public void periodic() {
        if (MANUAL_BULK_CACHING) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
        }
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        follower.update();
//        robot.turret.update();

        robot.shooter.setSpinning(gamepad1.right_trigger_pressed);

        robot.hoodGate.applyVelocityCompensation(
            robot.shooter.isSpinning(),
            robot.shooter.getTargetRpm(),
            robot.shooter.getCurrentRpm()
        );

        boolean trigger = gamepad1.left_trigger_pressed;
        boolean triggerJustPressed = trigger && !lastTrigger;
        lastTrigger = trigger;

        if (triggerJustPressed && !feedPending) {
            robot.hoodGate.openGate().schedule();
            feedTimer.reset();
            feedPending = true;
        }

        if (feedPending && feedTimer.milliseconds() >= GATE_OPEN_DELAY_MS) {
            if (x == 0) {
                robot.intakeTransfer.transfer().schedule();
                x = 1;
            } else {
                robot.intakeTransfer.feed().schedule();
                x = 0;
            }
            feedPending = false;
        }

        boolean readyNow = robot.shooter.atSpeed() && robot.turret.isOnTarget();
        if (readyNow != readyState) {
            readyState = readyNow;
            if (readyState) {
                gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS); // GREEN when ready!
                gamepad1.rumble(120);
            } else {
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS); // RED when aiming / spinning up
            }
        }

        if (telemetryTimer.milliseconds() >= TELEMETRY_PERIOD_MS) {
            telemetryTimer.reset();
            Telemetry.log("Status", "Running " + alliance);
            Pose pose = follower.pose();
            Telemetry.log("Pose", String.format("x=%.1f y=%.1f h=%.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            Telemetry.log("Goal", String.format("(%.0f, %.0f) %s", robot.goal.x(), robot.goal.y(), robot.goal.isUpper() ? "upper" : "lower"));
            Telemetry.log("Turret deg", String.format("want %.1f / now %.1f", robot.turret.getTargetDeg(), robot.turret.getCurrentDeg()));
            Telemetry.log("Gate pos", String.format("%.2f", robot.hoodGate.getGatePosition()));
            Telemetry.log("Hood comp", String.format("%.3f", robot.hoodGate.getLastCompensation()));
            Telemetry.log("Loop ms", String.format("%.1f", loopMs));
            Telemetry.log("Target RPM", String.format("%.1f", robot.shooter.getTargetRpm()));
            Telemetry.log("Current RPM", String.format("%.1f", robot.shooter.getCurrentRpm()));
            Telemetry.log("At speed", String.valueOf(robot.shooter.atSpeed()));
            Telemetry.log("Ready", String.valueOf(readyState));
        }
    }

    @Override
    public void end() {
        Telemetry.log("Status", "End");
        robot.shooter.setSpinning(false);
        robot.intakeTransfer.stop().schedule();
    }
}
