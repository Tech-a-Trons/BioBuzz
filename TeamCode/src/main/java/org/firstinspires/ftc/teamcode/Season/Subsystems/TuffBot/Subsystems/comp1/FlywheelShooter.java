package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.robot.Mechanism;

/**
 * Two-motor flywheel shooter, feedforward + PID velocity control (deg/sec).
 * "outtakeleft"  = commanded motor, the only one whose encoder is read.
 * "outtakeright" = follower, just gets the same computed power as the left.
 */
public class FlywheelShooter implements Mechanism {
    // ---- tuned constants (units in deg/sec) ----
    private static final double kS = 0.15;
    private static final double kV = 0.000027;
    private static final double kP = 0.00003;
    private static final double kI = 0.0000;
    private static final double kD = 0.0;

    private static final double TARGET_RPM = 1800;
    private static final double DEG_PER_SEC_PER_RPM = 6.0;
    private static final double MAX_INTEGRAL = 0;

    // Direction of each motor. If the right motor fights the left, flip RIGHT_SIGN.
    private static final double LEFT_SIGN = -1;
    private static final double RIGHT_SIGN = 1;

    private DcMotorEx motorLeft;   // commanded + encoder
    private DcMotorEx motorRight;  // follower
    private Follower follower;
    private GoalSelector goal;

    private final ElapsedTime timer = new ElapsedTime();
    private double integral = 0, lastError = 0;
    private boolean enabled = false;
    private double lastWritten = 2;

    private double targetRpm = TARGET_RPM, currentRpm = 0, rawDegPerSec = 0, power = 0;

    public void init(HardwareMap hardwareMap, Follower follower, GoalSelector goal) {
        this.follower = follower;
        this.goal = goal;

        motorLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Right motor never uses its encoder; it only mirrors the left's power.
        motorRight = hardwareMap.get(DcMotorEx.class, "outtakeright");

        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSpinning(boolean on) {
        if (on && !enabled) {
            integral = 0;
            lastError = 0;
            timer.reset();
        }
        enabled = on;
    }

    @Override
    public void periodic() {
        Mechanism.super.periodic();
        update();
    }

    private void update() {
        if (motorLeft == null || motorRight == null) return;
        if (!enabled) {
            integral = 0;
            lastError = 0;
            power = 0;
            targetRpm = TARGET_RPM;
            write(0);
            return;
        }

        targetRpm = TARGET_RPM;

        double dt = timer.seconds();
        timer.reset();

        // Only the left motor's encoder is used for feedback
        rawDegPerSec = motorLeft.getVelocity(AngleUnit.DEGREES);
        double currentDegPerSec = Math.abs(rawDegPerSec);
        currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;

        double targetDegPerSec = targetRpm * DEG_PER_SEC_PER_RPM;
        double errorDegPerSec = targetDegPerSec - currentDegPerSec;

        if (dt > 0 && Math.abs(errorDegPerSec) < 300) {
            integral = Range.clip(integral + errorDegPerSec * dt * kI, -MAX_INTEGRAL, MAX_INTEGRAL);
        } else {
            integral = 0;
        }
        double derivative = (dt > 0) ? (errorDegPerSec - lastError) / dt : 0;
        lastError = errorDegPerSec;

        power = kS + kV * targetDegPerSec + kP * errorDegPerSec + integral + kD * derivative;
        power = Range.clip(power, 0, 1);

        write(power);
    }

    /** p is the unsigned shooter power (0..1); each motor applies its own direction. */
    private void write(double p) {
        if (Math.abs(p - lastWritten) > 0.002 || (p == 0 && lastWritten != 0)) {
            motorLeft.setPower(LEFT_SIGN * p);
            motorRight.setPower(RIGHT_SIGN * p);
            lastWritten = p;
        }
    }

    public boolean isSpinning()   { return enabled; }
    public boolean atSpeed()      { return enabled && Math.abs(targetRpm - currentRpm) < 0.10 * targetRpm; }
    public double getTargetRpm()  { return targetRpm; }
    public double getCurrentRpm() { return currentRpm; }
    public double getRawVelocity(){ return rawDegPerSec; }
    public double getPower()      { return power; }
}