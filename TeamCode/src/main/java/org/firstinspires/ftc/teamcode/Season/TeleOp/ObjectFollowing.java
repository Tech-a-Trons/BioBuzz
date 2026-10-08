package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.ProtoIntake;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;

/**
 * ALL-IN-ONE FILE: Pedro follower init + pose cache + Limelight scan-and-intake (Ivy).
 *
 * AUTO:
 *   follower = ObjectFollowing.PedroInit.forAuto(hardwareMap, new Pose(72, 72, Math.toRadians(90)));
 *   periodic(): follower.update();
 *   start():    ObjectFollowing.buildForPollen(follower, limelight, intake).schedule();
 *   end():      ObjectFollowing.PoseCache.save(follower.pose());
 *
 * PIPELINES:
 *   - Pollen = 1
 *   - Red Nectar = 4
 *   - Blue Nectar = 5
 */
public final class ObjectFollowing {
    private ObjectFollowing() {}

    // ================= PIPELINES =================
    public static final int PIPELINE_POLLEN = 1;
    public static final int PIPELINE_RED_NECTAR = 4;
    public static final int PIPELINE_BLUE_NECTAR = 5;

    public static int PIPELINE = PIPELINE_POLLEN;

    public static double SCAN_TURN_POWER = 0.30;
    public static double SCAN_MAX_DEG = 360;

    public static double ALIGN_KP = 0.025;
    public static double ALIGN_MIN_POWER = 0.08;
    public static double ALIGN_TOL_DEG = 2.0;
    public static double ALIGN_LOST_MS = 400;

    public static double APPROACH_POWER = 0.4;
    public static double APPROACH_STEER_KP = 0.02;
    public static double CLOSE_ENOUGH_AREA = 8.0;   // ta %. TUNE THIS
    public static double LOST_CLOSE_MS = 350;
    public static double APPROACH_TIMEOUT_MS = 5000;

    public static double PUSH_POWER = 0.2;
    public static double FINISH_MS = 900;
    // ==========================================

    // ============== POSE CACHE ================
    /** Remembers the pose between OpModes (auto -> teleop). Lives until the RC app restarts. */
    public static final class PoseCache {
        private PoseCache() {}

        /** Ignore cached poses older than this. */
        public static long MAX_AGE_MS = 5 * 60 * 1000;

        private static double x, y, heading;
        private static long savedAtMs = -1;

        /** Copies the pose (Pose is mutable; never keep the follower's own reference). */
        public static void save(Pose pose) {
            if (pose == null) return;
            x = pose.x();
            y = pose.y();
            heading = pose.heading();
            savedAtMs = System.currentTimeMillis();
        }

        public static boolean hasFresh() {
            return savedAtMs >= 0 && System.currentTimeMillis() - savedAtMs <= MAX_AGE_MS;
        }

        public static Pose getOr(Pose fallback) {
            return hasFresh() ? new Pose(x, y, heading) : fallback;
        }

        public static void clear() { savedAtMs = -1; }
    }

    // ============== PEDRO INIT ================
    /** Builds the Pedro follower and seeds its starting pose. */
    public static final class PedroInit {
        private PedroInit() {}

        /** AUTO: always start from the known field pose; clears any stale cache. */
        public static Follower forAuto(HardwareMap hw, Pose startPose) {
            PoseCache.clear();
            Follower f = Constants.create(hw);
            if (f != null) {
                f.setPose(startPose);
                f.update(); // avoids a null pose before the first loop
            }
            return f;
        }

        /** TELEOP: resume from the cached pose, or fallbackPose if none is fresh. */
        public static Follower fromCache(HardwareMap hw, Pose fallbackPose) {
            Follower f = Constants.create(hw);
            if (f != null) {
                f.setPose(PoseCache.getOr(fallbackPose));
                f.update();
            }
            return f;
        }
    }

    // ============== THE COMMAND ===============
    private enum State { SCAN, ALIGN, APPROACH, INTAKE, DONE }

    public static Command build(Follower follower, Limelight3A limelight, ProtoIntake intake) {
        return build(follower, limelight, intake, PIPELINE);
    }

    public static Command build(Follower follower, Limelight3A limelight, ProtoIntake intake, int pipeline) {
        Run run = new Run(follower, limelight, intake, pipeline);
        return Command.build()
                .setStart(run::start)
                .setExecute(run::update)
                .setDone(run::isDone)
                .setEnd(endCondition -> run.end())
                .requiring(intake);
    }

    public static Command buildForPollen(Follower follower, Limelight3A limelight, ProtoIntake intake) {
        return build(follower, limelight, intake, PIPELINE_POLLEN);
    }

    public static Command buildForRedNectar(Follower follower, Limelight3A limelight, ProtoIntake intake) {
        return build(follower, limelight, intake, PIPELINE_RED_NECTAR);
    }

    public static Command buildForBlueNectar(Follower follower, Limelight3A limelight, ProtoIntake intake) {
        return build(follower, limelight, intake, PIPELINE_BLUE_NECTAR);
    }

    /** Per-call state holder so every build() returns an independent command. */
    private static final class Run {
        final Follower follower;
        final Limelight3A limelight;
        final ProtoIntake intake;
        final int targetPipeline;

        State state = State.SCAN;
        final ElapsedTime stateTimer = new ElapsedTime();
        final ElapsedTime lostTimer = new ElapsedTime();
        double scanTurned, lastHeading;
        Command runningIntake;

        Run(Follower f, Limelight3A l, ProtoIntake i, int pipeline) {
            follower = f; limelight = l; intake = i; targetPipeline = pipeline;
        }

        void start() {
            if (limelight != null) {
                limelight.pipelineSwitch(targetPipeline);
                limelight.setPollRateHz(100);
                limelight.start();
            }
            scanTurned = 0;
            lastHeading = follower != null ? follower.pose().heading() : 0;
            enter(State.SCAN);
        }

        boolean isDone() { return state == State.DONE; }

        void update() {
            if (limelight == null || follower == null) {
                enter(State.DONE);
                return;
            }
            LLResult r = limelight.getLatestResult();
            boolean seen = r != null && r.isValid();
            double tx = seen ? r.getTx() : 0;
            double ta = seen ? r.getTa() : 0;
            if (seen) lostTimer.reset();

            switch (state) {
                case SCAN: {
                    double h = follower.pose().heading();
                    scanTurned += Math.abs(Math.toDegrees(wrap(h - lastHeading)));
                    lastHeading = h;
                    if (seen) { drive(0, 0); enter(State.ALIGN); }
                    else if (scanTurned >= SCAN_MAX_DEG) { drive(0, 0); enter(State.DONE); }
                    else drive(0, SCAN_TURN_POWER);
                    break;
                }
                case ALIGN: {
                    if (!seen) {
                        drive(0, 0);
                        if (lostTimer.milliseconds() > ALIGN_LOST_MS) {
                            scanTurned = 0;
                            lastHeading = follower.pose().heading();
                            enter(State.SCAN);
                        }
                    } else if (Math.abs(tx) <= ALIGN_TOL_DEG) {
                        drive(0, 0);
                        enter(State.APPROACH);
                        startIntake();
                    } else {
                        double p = -tx * ALIGN_KP;
                        if (Math.abs(p) < ALIGN_MIN_POWER) p = Math.copySign(ALIGN_MIN_POWER, p);
                        drive(0, clamp(p, -0.4, 0.4));
                    }
                    break;
                }
                case APPROACH: {
                    boolean close = seen && ta >= CLOSE_ENOUGH_AREA;
                    boolean lostClose = !seen && lostTimer.milliseconds() > LOST_CLOSE_MS
                            && stateTimer.milliseconds() > 500;
                    if (close || lostClose) enter(State.INTAKE);
                    else if (stateTimer.milliseconds() > APPROACH_TIMEOUT_MS) { drive(0, 0); enter(State.DONE); }
                    else if (seen) drive(APPROACH_POWER, clamp(-tx * APPROACH_STEER_KP, -0.2, 0.2));
                    else drive(APPROACH_POWER, 0);
                    break;
                }
                case INTAKE: {
                    drive(PUSH_POWER, 0);
                    if (stateTimer.milliseconds() > FINISH_MS) { drive(0, 0); enter(State.DONE); }
                    break;
                }
                default: break;
            }
        }

        void end() {
            if (runningIntake != null) runningIntake.cancel();
            if (intake != null) intake.stop().schedule();
            if (follower != null) {
                follower.hold(follower.pose());
                PoseCache.save(follower.pose());
            }
            if (limelight != null) {
                limelight.stop();
            }
        }

        void startIntake() {
            if (intake != null) {
                runningIntake = intake.run();
                runningIntake.schedule();
            }
        }

        void enter(State s) { state = s; stateTimer.reset(); }
        void drive(double fwd, double turn) {
            if (follower != null && follower.drivetrain != null) {
                follower.drivetrain.drive(new DrivePowers(fwd, 0, turn), false);
            }
        }
    }

    private static double wrap(double rad) {
        double r = rad;
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
