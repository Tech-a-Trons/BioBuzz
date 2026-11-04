package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class EasyTurret {
    private static final double MOTOR_CPR = 537.7;
    private static final double GEAR_RATIO = 3.0;   // motor rotations per turret rotation
    private static final double TICKS_PER_DEG = MOTOR_CPR * GEAR_RATIO / 360.0;
    private static final double MIN_DEG = -180, MAX_DEG = 180; // set to your real cable/travel limits
    private static final double MAX_POWER = 0.8;
    private static final int    TOLERANCE_TICKS = 6;
    private static final int    RETARGET_TICKS  = 4;

    // ---- the two things that most often break turret tracking ----
    /** +1 if positive encoder ticks turn the turret counter-clockwise (seen from above), -1 if clockwise. */
    private static final double TURRET_SIGN = 1;
    /** Turret angle (deg) at encoder 0 relative to robot forward. 0 = turret points forward when the encoder is reset. */
    private static final double ZERO_OFFSET_DEG = 0;

    private DcMotorEx turret;
    private Follower follower;
    private GoalSelector goal;
    private static final double ON_TARGET_DEG = 2.0;
    private int lastTargetTicks = Integer.MIN_VALUE;

    // debug values
    private double desiredDeg, currentDeg, targetDeg;

    /** Turret must be physically centered (pointing at ZERO_OFFSET_DEG) when this runs. */
    public void init(HardwareMap hardwareMap, Follower follower, GoalSelector goal) {
        this.follower = follower;
        this.goal = goal;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setTargetPositionTolerance(TOLERANCE_TICKS);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(MAX_POWER);
    }

    public void update() {
        if (turret == null) return;

        Pose p = follower.pose();
        double x = p.x(), y = p.y(), heading = p.heading();

        goal.update(y);
        double fieldAngle = Math.atan2(goal.y() - y, goal.x() - x);

        // Angle of the goal relative to the robot's front, CCW positive
        double relDeg = Math.toDegrees(AngleUnit.normalizeRadians(fieldAngle - heading));
        desiredDeg = AngleUnit.normalizeDegrees(TURRET_SIGN * (relDeg - ZERO_OFFSET_DEG));

        // Equivalent angle (+-360) inside travel limits that's closest to where we are
        currentDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        targetDeg = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < MIN_DEG || cand > MAX_DEG) continue;
            double d = Math.abs(cand - currentDeg);
            if (d < bestDist) { bestDist = d; targetDeg = cand; }
        }
        if (Double.isNaN(targetDeg)) targetDeg = Range.clip(desiredDeg, MIN_DEG, MAX_DEG);

        int targetTicks = (int) Math.round(targetDeg * TICKS_PER_DEG);
        if (Math.abs(targetTicks - lastTargetTicks) >= RETARGET_TICKS) {
            turret.setTargetPosition(targetTicks);
            lastTargetTicks = targetTicks;
        }
    }

    public Command track() { return infinite(this::update); }

    /** True when the turret is within ON_TARGET_DEG of where it is trying to point. */
    public boolean isOnTarget() { return turret != null && Math.abs(currentDeg - targetDeg) <= ON_TARGET_DEG; }

    public double getDesiredDeg() { return desiredDeg; }
    public double getCurrentDeg() { return currentDeg; }
    public double getTargetDeg()  { return targetDeg; }
}