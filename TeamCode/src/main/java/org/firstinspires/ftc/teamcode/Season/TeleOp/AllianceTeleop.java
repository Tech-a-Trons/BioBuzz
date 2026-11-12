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

    // Used only if Auto didn't leave a pose in PoseStorage. Set to where you really place the robot.
    // Heading is radians; blue is the mirror of red (x -> 141.5 - x, heading -> pi - heading).
    private static final Pose FALLBACK_RED  = new Pose(70.75, 8.6, Math.toRadians(270));
    private static final Pose FALLBACK_BLUE = new Pose(141.5 - 70.75, 70.75, Math.PI - Math.toRadians(270));

    // Relocalization pose for dpad right (START_POSE from TurretOp, which was set up for BLUE).
    // Red is the mirror of blue.
    private static final Pose START_BLUE = new Pose(70.75, 8.6, Math.toRadians(270));
    private static final Pose START_RED  = new Pose(70.75, 8.6, Math.toRadians(270));

    // Robot heading (radians) that is "forward" for the driver at the start. Set this to the direction the
    // driver looks across the field. Press BACK any time to re-zero it to the robot's current facing.
    private static final double DRIVER_FORWARD_RED  = Math.toRadians(270);
    private static final double DRIVER_FORWARD_BLUE = Math.toRadians(270);
    private static final long READY_HOLD_MS = 120;     // must be ready this long before the buzz
    private static final long GATE_OPEN_DELAY_MS = 250; // same as TurretOp GATE_FEED_DELAY_SEC (0.25 s)
    private static final long TELEMETRY_PERIOD_MS = 100;
    // Set false if NextFTC already clears the hub bulk cache for you
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

        // Continuous commands
        robot.drive.setHeadingSource(() -> follower.pose().heading());
        robot.drive.setForwardHeading("BLUE".equals(alliance) ? DRIVER_FORWARD_BLUE : DRIVER_FORWARD_RED);
        robot.drive.startDrive(gamepad1);
        //robot.turret.track().schedule();
//        robot.flower.f1start();
//        robot.flower.f2start();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        // Intake / transfer
        gp1.a().onTrue(robot.intakeTransfer.collect());
        gp1.a().onTrue(instant(() -> feedPending = false));
        gp1.rightBumper().onTrue(robot.intakeTransfer.feedSlow());
        gp1.rightBumper().onTrue(robot.hoodGate.closeGate());   // TurretOp sets gate to 0.22 (closed) here
        gp1.leftBumper().onTrue(robot.intakeTransfer.stop());
        gp1.leftBumper().onTrue(robot.hoodGate.closeGate());
        gp1.leftBumper().onTrue(instant(() -> feedPending = false));

        //gp1.x().onTrue(robot.intakeTransfer.transfer());

        // Hood / gate presets
        gp1.dpadUp().onTrue(robot.hoodGate.presetUp());
        gp1.dpadDown().onTrue(robot.hoodGate.presetDown());

        //Flower Ramp controls
//        gp1.dpadLeft().onTrue(robot.flower.f1down());
//        gp1.dpadLeft().onTrue(robot.flower.f2down());
//        gp1.dpadRight().onTrue(robot.flower.f1start());
//        gp1.dpadRight().onTrue(robot.flower.f2start());

        // Relocalize (same as TurretOp dpad right)
        gp1.dpadRight().onTrue(instant(() ->
                follower.setPose("BLUE".equals(alliance) ? START_BLUE : START_RED)));

        // Shooter mode: fixed RPM <-> distance-based RPM
        //gp1.x().onTrue(robot.shooter.toggleOdo());

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

        // Shooter spins while the right trigger is held
        robot.shooter.setSpinning(gamepad1.right_trigger_pressed);

        boolean trigger = gamepad1.left_trigger_pressed;
        boolean triggerJustPressed = trigger && !lastTrigger;   // rising edge only
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

        // Ready-to-shoot feedback: shooter at speed AND turret on target -> short buzz + green light
        boolean readyNow = robot.shooter.atSpeed() && robot.turret.isOnTarget();
        if (!readyNow) {
            readyTimer.reset();
            if (readyState) {
                readyState = false;
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
        } else if (!readyState && readyTimer.milliseconds() >= READY_HOLD_MS) {
            readyState = true;
            gamepad1.rumble(120);
            gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (telemetryTimer.milliseconds() >= TELEMETRY_PERIOD_MS) {
            telemetryTimer.reset();
            Telemetry.log("Status", "Running " + alliance);
            Pose pose = follower.pose();
            Telemetry.log("Pose", String.format("x=%.1f y=%.1f h=%.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            Telemetry.log("Goal", String.format("(%.0f, %.0f) %s", robot.goal.x(), robot.goal.y(), robot.goal.isUpper() ? "upper" : "lower"));
            Telemetry.log("Turret deg", String.format("want %.1f / now %.1f", robot.turret.getTargetDeg(), robot.turret.getCurrentDeg()));
            Telemetry.log("Gate pos", String.format("%.2f", robot.hoodGate.getGatePosition()));
            Telemetry.log("Loop ms", String.format("%.1f", loopMs));
            Telemetry.log("Shot mode", robot.shooter.isOdoMode() ? "ODO" : "FIXED");
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