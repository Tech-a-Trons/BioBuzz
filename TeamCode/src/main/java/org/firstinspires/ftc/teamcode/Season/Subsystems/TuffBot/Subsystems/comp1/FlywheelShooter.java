package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.robot.Mechanism;

/**
 * Flywheel shooter with properly scaled feedforward + PID velocity control (in deg/sec).
 */
public class FlywheelShooter implements Mechanism {
    // ---- tuned constants (units in deg/sec) ----
    private static final double kS = 0.02;
    private static final double kV = 0.0025;   // ~0.45 power at 1800 deg/s (300 RPM)
    private static final double kP = 0.002;    // smooth velocity error correction
    private static final double kI = 0.00001;
    private static final double kD = 0.0;

    private static final double TARGET_RPM = 25.0;                      // 300 RPM
    private static final double DEG_PER_SEC_PER_RPM = 6.0;               // 1 RPM = 6 deg/sec
    private static final double MAX_INTEGRAL = 0.2;
    private static final double SHOOTER_SIGN = -1;

    private DcMotorEx motor;
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

        motor = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        if (motor == null) return;
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

        rawDegPerSec = motor.getVelocity(AngleUnit.DEGREES);
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

        // Feedforward + smooth PID velocity loop
        power = kS + kV * targetDegPerSec + kP * errorDegPerSec + integral + kD * derivative;
        power = Range.clip(power, 0, 1);

        write(SHOOTER_SIGN * power);
    }

    private void write(double p) {
        if (Math.abs(p - lastWritten) > 0.002 || (p == 0 && lastWritten != 0)) {
            motor.setPower(p);
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
