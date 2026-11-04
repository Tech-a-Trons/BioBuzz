//package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;
//
//import static dev.nextftc.units.Units.Degrees;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.ivy.Command;
//
//import java.util.function.DoubleSupplier;
//import java.util.function.Supplier;
//
//import dev.nextftc.hardware.NextMotor;
//import dev.nextftc.robot.Mechanism;
//
//// NextFTC v2 imports: package names are from memory, so let Android Studio resolve them (Alt+Enter).
//
///**
// * One-motor turret that keeps aiming at a field point, even while the robot circles it.
// *
// * Angle 0 = turret facing robot-forward (where it sat at robot startup), positive = counter-clockwise,
// * same as Pedro heading.
// *
// * Add it to your NextRobot's getMechanisms() set. periodic() and the default command only run for
// * mechanisms in that set.
// *
// *   Turret turret = new Turret(() -> follower.getPose());
// *   gamepad.leftStickX -> turret.manual(() -> gamepad2.left_stick_x)   // overrides tracking while held
// */
//public class Turret implements Mechanism {
//
//    // ---------- Config ----------
//    private static final String MOTOR_NAME = "turretMotor";
//    private static final double MOTOR_TICKS_PER_REV = 537.7;   // goBILDA 312 RPM. Change for your motor.
//    private static final double MOTOR_REVS_PER_TURRET_REV = 3.0; // 1:3 ratio = motor turns 3x for every 1 turret turn
//    private static final double TICKS_PER_TURRET_REV = MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_TURRET_REV;
//
//    // Exactly 360 degrees of travel: -180 to +180, with 0 = forward.
//    // Slip ring? Set LIMITED = false and it spins freely, always taking the shortest path.
//    private static final boolean LIMITED = true;
//    private static final double MIN_DEG = -180;
//    private static final double MAX_DEG = 180;
//
//    // Position PID gains for NextMotor's built-in controller. Tune these.
//    private static final double KP = 0.1;
//    private static final double KD = 0.0;
//
//    // Target in Pedro field coordinates (inches). Replace with your hive position.
//    public static double xt = 60;
//    public static double yt = 60;
//    public static double offsetDeg = 0; // trim for shooter misalignment
//
//    // ---------- Hardware ----------
//    // anglePerCount tells NextMotor how many degrees one encoder tick is.
//    // Add motor.setDirection(Direction.REVERSE) if positive turns the turret clockwise.
//    private final NextMotor motor =
//            new NextMotor(MOTOR_NAME, Degrees.of(360.0 / TICKS_PER_TURRET_REV));
//
//    private final Supplier<Pose> pose;
//    private double commandedDeg = 0; // last angle we asked for (unwrapped)
//
//    public Turret(Supplier<Pose> pose) {
//        this.pose = pose;
//        motor.getPositionConstants().withP(KP).withD(KD);
//    }
//
//    // ---------- Commands ----------
//    /** Runs whenever nothing else is using the turret: keeps aiming at the target. */
//    @Override
//    public Command getDefaultCommand() {
//        return infinite(this::track);
//    }
//
//    /** Manual override while scheduled (e.g. while a button is held). Positive = counter-clockwise. */
//    public Command manual(DoubleSupplier power) {
//        return infinite(() -> motor.setThrottle(power.getAsDouble()));
//    }
//
//    // ---------- API ----------
//    public void setTarget(double x, double y) {
//        xt = x;
//        yt = y;
//    }
//
//    // ---------- Internals ----------
//    private void track() {
//        Pose p = pose.get();
//        if (p == null) return;
//
//        // Which way should the turret point relative to the robot?
//        double fieldDeg = Math.toDegrees(Math.atan2(yt - p.getY(), xt - p.getX()));
//        double desired = wrap(fieldDeg - Math.toDegrees(p.getHeading()) + offsetDeg);
//
//        if (LIMITED) {
//            // Pick the right way around (this is what makes full 360 circling work).
//            commandedDeg = closestLegal(desired, commandedDeg);
//            motor.setPositionSetpoint(Degrees.of(commandedDeg));
//        } else {
//            // NextMotor takes the shortest path to an angle in [-180, 180].
//            motor.setAbsolutePositionSetpoint(Degrees.of(desired));
//        }
//    }
//
//    /** Wraps to (-180, 180]. */
//    private static double wrap(double a) {
//        a %= 360;
//        if (a > 180) a -= 360;
//        else if (a <= -180) a += 360;
//        return a;
//    }
//
//    /** Of desired +/- 360, the angle inside [MIN, MAX] that is closest to the last commanded angle. */
//    private static double closestLegal(double desired, double current) {
//        double best = Double.NaN;
//        for (int k = -1; k <= 1; k++) {
//            double c = desired + 360 * k;
//            if (c < MIN_DEG || c > MAX_DEG) continue;
//            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) best = c;
//        }
//        return Double.isNaN(best) ? Math.max(MIN_DEG, Math.min(MAX_DEG, desired)) : best;
//    }
//}