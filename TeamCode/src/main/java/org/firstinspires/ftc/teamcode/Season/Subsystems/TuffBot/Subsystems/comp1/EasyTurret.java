package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Turret tracker with feedforward.
 *
 * Control structure (runs every loop, our own math, motor is RUN_WITHOUT_ENCODER):
 *
 *   1. Aim: predict the pose forward by LATENCY_S, compute the turret angle that points at the goal.
 *   2. Feedforward rate: how fast that angle is changing because the robot is rotating
 *      (-robot omega) and translating (goal bearing rate). The turret moves at this rate
 *      WITHOUT needing any position error, so it doesn't lag behind while driving/spinning.
 *   3. Position loop (sqrt-profile): error -> velocity command. Far away it brakes at DECEL_DEG_S2
 *      so big slews (goal switch) are fast and don't overshoot; near zero it is linear.
 *   4. Velocity loop: power = kS*sign + kV*vCmd + kVP*(vCmd - vMeasured) (+ small integral).
 */
public class EasyTurret {
    // ---- geometry ----
    private static final double MOTOR_CPR = 537.7;
    private static final double GEAR_RATIO = 3.0;   // motor rotations per turret rotation
    private static final double TICKS_PER_DEG = MOTOR_CPR * GEAR_RATIO / 360.0;
    private static final double MIN_DEG = -180, MAX_DEG = 180; // set to your real cable/travel limits
    private static final double LIMIT_MARGIN_DEG = 5.0;        // cut power if we get this far past a limit
    // With exactly +-180 there is NO overlap: every time the goal crosses the robot's rear the target jumps
    // 360 deg and the turret has to unwind the whole way. If your cables allow more than 360 deg of travel,
    // widen MIN/MAX (e.g. -200..200) so there is overlap and the turret only unwinds when it truly must.
    private static final double UNWIND_ACCEL_DEG_S2 = 4000;    // gentler accel during a full unwind (less belt/gear skip)

    // ---- the two things that most often break turret tracking ----
    /** +1 if positive encoder ticks turn the turret counter-clockwise (seen from above), -1 if clockwise. */
    private static final double TURRET_SIGN = 1;
    /** Turret angle (deg) at encoder 0 relative to robot forward. 0 = turret points forward when the encoder is reset. */
    private static final double ZERO_OFFSET_DEG = 0;

    // ---- motion limits (deg/s, deg/s^2) ----
    private static final double MAX_POWER = 0.8;
    private static final double MAX_VEL_DEG_S = 420;     // keep below (MAX_POWER - kS) / kV
    private static final double MAX_ACCEL_DEG_S2 = 6000; // slew limit on the velocity command
    private static final double DECEL_DEG_S2 = 2500;     // braking used by the sqrt position profile
    private static final double LINEAR_ZONE_DEG = 4.0;   // inside this the position loop is linear

    // ---- gains (tune in this order: kS, kV, kVP, then LATENCY_S) ----
    private static final double kS = 0.04;      // power to just start moving
    private static final double kV = 0.0017;    // power per deg/s (5203 312rpm / 3:1 tops out ~620 deg/s unloaded)
    private static final double kVP = 0.0010;   // power per deg/s of velocity error
    private static final double kI = 0.0;       // optional, only acts inside I_ZONE_DEG
    private static final double I_ZONE_DEG = 3.0, MAX_INTEGRAL = 0.1;
    private static final double DEADBAND_DEG = 0.25;     // no kS push inside this

    // ---- estimation ----
    /** Time between "pose read" and the turret actually responding. Raise until tracking while spinning stops lagging. */
    private static final double LATENCY_S = 0.07;
    private static final double OMEGA_ALPHA = 0.4;       // 1 = no filtering, smaller = smoother but laggier
    private static final double VEL_ALPHA = 0.4;
    private static final double TURRET_VEL_ALPHA = 0.5;

    // ---- ready check ----
    private static final double ON_TARGET_DEG = 2.0;
    private static final double ON_TARGET_VEL_DEG_S = 40;

    private static final double KP_POS = Math.sqrt(DECEL_DEG_S2 / LINEAR_ZONE_DEG); // continuous at zone edge

    private DcMotorEx turret;
    private Follower follower;
    private GoalSelector goal;
    private final ElapsedTime timer = new ElapsedTime();

    // pose-derivative estimates
    private boolean haveLast = false;
    private double lastX, lastY, lastH;
    private double omega = 0, vx = 0, vy = 0;   // rad/s, in/s

    // controller state
    private double vCmd = 0, integral = 0, lastPower = 2;
    private double vMeas = 0;
    private double lastTargetDeg = Double.NaN;
    private boolean unwinding = false;

    // debug values
    private double desiredDeg, currentDeg, targetDeg, errorDeg, ffRate, power;
    private int ticks;

    /** Turret must be physically centered (pointing at ZERO_OFFSET_DEG) when this runs. */
    public void init(HardwareMap hardwareMap, Follower follower, GoalSelector goal) {
        this.follower = follower;
        this.goal = goal;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);
        timer.reset();
    }

    /** Call once per loop, after follower.update(). */
    public void update() {
        if (turret == null) return;

        double dt = timer.seconds();
        if (haveLast && dt < 0.002) return; // called twice in one loop: skip so we never divide by ~0
        timer.reset();

        Pose p = follower.pose();
        double x = p.x(), y = p.y(), h = p.heading();

        // ---- estimate robot velocity from pose (filtered) ----
        if (haveLast && dt < 0.1) {
            double dh = AngleUnit.normalizeRadians(h - lastH);
            omega = omega + OMEGA_ALPHA * (dh / dt - omega);
            vx = vx + VEL_ALPHA * ((x - lastX) / dt - vx);
            vy = vy + VEL_ALPHA * ((y - lastY) / dt - vy);
        }
        lastX = x; lastY = y; lastH = h;
        haveLast = true;

        // ---- predict pose forward to cover sensing + actuation latency ----
        double hp = h + omega * LATENCY_S;
        double xp = x + vx * LATENCY_S;
        double yp = y + vy * LATENCY_S;

        goal.update(y);
        double gx = goal.x() - xp, gy = goal.y() - yp;
        double fieldAngle = Math.atan2(gy, gx);

        // Angle of the goal relative to the robot's front, CCW positive
        double relDeg = Math.toDegrees(AngleUnit.normalizeRadians(fieldAngle - hp));
        desiredDeg = AngleUnit.normalizeDegrees(TURRET_SIGN * (relDeg - ZERO_OFFSET_DEG));

        // Rate the desired turret angle is changing at:
        //   goal bearing rate (from robot translation)  minus  robot rotation rate
        double r2 = gx * gx + gy * gy;
        double bearingRate = (r2 > 1.0) ? (gy * vx - gx * vy) / r2 : 0; // rad/s
        ffRate = TURRET_SIGN * Math.toDegrees(bearingRate - omega);

        // ---- measure turret ----
        ticks = turret.getCurrentPosition();
        currentDeg = ticks / TICKS_PER_DEG;
        vMeas = vMeas + TURRET_VEL_ALPHA * (turret.getVelocity() / TICKS_PER_DEG - vMeas);

        // Equivalent angle (+-360) inside travel limits that's closest to where we are
        targetDeg = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < MIN_DEG || cand > MAX_DEG) continue;
            double d = Math.abs(cand - currentDeg);
            if (d < bestDist) { bestDist = d; targetDeg = cand; }
        }
        if (Double.isNaN(targetDeg)) {
            targetDeg = Range.clip(desiredDeg, MIN_DEG, MAX_DEG);
            ffRate = 0; // can't follow the goal, don't push
        }
        // A target jump of more than half a turn means the wrap just happened: we are now unwinding
        if (!Double.isNaN(lastTargetDeg) && Math.abs(targetDeg - lastTargetDeg) > 180) unwinding = true;
        lastTargetDeg = targetDeg;
        // Don't feed forward into a travel limit
        if ((targetDeg >= MAX_DEG - 0.01 && ffRate > 0) || (targetDeg <= MIN_DEG + 0.01 && ffRate < 0)) ffRate = 0;

        // ---- position loop -> velocity command ----
        errorDeg = targetDeg - currentDeg;
        double absErr = Math.abs(errorDeg);
        double vPos = (absErr <= LINEAR_ZONE_DEG)
                ? KP_POS * errorDeg
                : Math.copySign(Math.min(MAX_VEL_DEG_S,
                Math.sqrt(2.0 * DECEL_DEG_S2 * (absErr - LINEAR_ZONE_DEG / 2.0))), errorDeg);

        double vDes = Range.clip(ffRate + vPos, -MAX_VEL_DEG_S, MAX_VEL_DEG_S);
        if (unwinding && absErr < 15) unwinding = false;
        double maxStep = (unwinding ? UNWIND_ACCEL_DEG_S2 : MAX_ACCEL_DEG_S2) * Math.min(dt, 0.05);
        vCmd = Range.clip(vDes, vCmd - maxStep, vCmd + maxStep);

        // ---- velocity loop -> motor power ----
        double friction;
        if (Math.abs(vCmd) > 2.0) friction = Math.copySign(kS, vCmd);
        else if (absErr > DEADBAND_DEG) friction = Math.copySign(kS, errorDeg);
        else friction = 0;

        if (absErr < I_ZONE_DEG) {
            integral = Range.clip(integral + kI * errorDeg * Math.min(dt, 0.05), -MAX_INTEGRAL, MAX_INTEGRAL);
        } else {
            integral = 0;
        }

        power = friction + kV * vCmd + kVP * (vCmd - vMeas) + integral;
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        // Hard safety: never keep driving further past a travel limit
        if (currentDeg > MAX_DEG + LIMIT_MARGIN_DEG && power > 0) power = 0;
        if (currentDeg < MIN_DEG - LIMIT_MARGIN_DEG && power < 0) power = 0;

        write(power);
    }

    // skip the hardware call if the power barely changed
    private void write(double pw) {
        if (Math.abs(pw - lastPower) > 0.002 || (pw == 0 && lastPower != 0)) {
            turret.setPower(pw);
            lastPower = pw;
        }
    }

    public Command track() { return infinite(this::update); }

    /** True when the turret is within ON_TARGET_DEG of where it should point and moving with the goal. */
    public boolean isOnTarget() {
        return turret != null
                && Math.abs(errorDeg) <= ON_TARGET_DEG
                && Math.abs(vMeas - ffRate) <= ON_TARGET_VEL_DEG_S;
    }

    public double getDesiredDeg() { return desiredDeg; }
    public double getCurrentDeg() { return currentDeg; }
    public double getTargetDeg()  { return targetDeg; }
    public double getEncoderTicks() { return ticks; }

    // extra tuning telemetry
    public double getErrorDeg()                 { return errorDeg; }
    public double getFeedforwardDegPerSec()     { return ffRate; }
    public double getVelocityDegPerSec()        { return vMeas; }
    public double getRobotOmegaDegPerSec()      { return Math.toDegrees(omega); }
    public double getPower()                    { return power; }
    public boolean isUnwinding()                { return unwinding; }
}