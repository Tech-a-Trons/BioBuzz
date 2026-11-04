package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.robot.Mechanism;

/**
 * Flywheel with our own feedforward + PID (constants from the old MecanumTeleOp)
 * and the odometry-based RPM from OdoPID merged in. Call update() every loop.
 */
public class FlywheelShooter implements Mechanism {
    // ---- tune these ----
    private static final double kS = 0.12;
    private static final double kV = 0.001;
    private static final double kP = 0.027;   // error is in deg/s
    private static final double kI = 0.0;
    private static final double kD = 0.03;

    private static final double FIXED_RPM = 23;            // old TARGET_RPM
    private static final double RPM_PER_INCH = 1.13636;    // from OdoPID
    private static final double DEG_PER_SEC_PER_RPM = 6.0;
    private static final double MAX_INTEGRAL = 0.3;
    private static final double SHOOTER_SIGN = -1;         // reverses final motor power only

    private DcMotorEx motor;
    private Follower follower;
    private GoalSelector goal;

    private final ElapsedTime timer = new ElapsedTime();
    private double integral = 0, lastError = 0;
    private boolean enabled = false, useOdo = false;
    private double lastWritten = 2;

    // telemetry values
    private double targetRpm = FIXED_RPM, currentRpm = 0, rawDegPerSec = 0, power = 0, lastDist = 0;

    public void init(HardwareMap hardwareMap, Follower follower, GoalSelector goal) {
        this.follower = follower;
        this.goal = goal;

        motor = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Spin up / spin down. Safe to call every loop. */
    public void setSpinning(boolean on) {
        if (on && !enabled) {
            integral = 0;
            lastError = 0;
            timer.reset();
        }
        enabled = on;
    }

    /** Toggle between fixed RPM and distance-based RPM. */
    public Command toggleOdo() {
        return instant(() -> {
            useOdo = !useOdo;
            integral = 0;
            lastError = 0;
        });
    }

    /** NextFTC calls this every loop; it runs the PID. */
    @Override
    public void periodic() {
        Mechanism.super.periodic();
        update();
    }

    private void update() {
        if (motor == null) return; // periodic() can run before init()
        if (!enabled) {
            integral = 0;
            lastError = 0;
            power = 0;
            targetRpm = FIXED_RPM;
            write(0);
            return;
        }

        if (useOdo) {
            Pose p = follower.pose();
            goal.update(p.y());
            lastDist = Math.hypot(goal.x() - p.x(), goal.y() - p.y());
            targetRpm = RPM_PER_INCH * lastDist;
        } else {
            targetRpm = FIXED_RPM;
        }

        double dt = timer.seconds();
        timer.reset();

        rawDegPerSec = motor.getVelocity(AngleUnit.DEGREES);
        double currentDegPerSec = Math.abs(rawDegPerSec);
        currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;

        double error = targetRpm * DEG_PER_SEC_PER_RPM - currentDegPerSec;

        if (dt > 0) {
            integral = Range.clip(integral + error * dt * kI, -MAX_INTEGRAL, MAX_INTEGRAL);
        }
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        power = kS + kV * targetRpm + kP * error + integral + kD * derivative;
        power = Range.clip(power, 0, 1);

        write(SHOOTER_SIGN * power);
    }

    // skip the hardware call if the power barely changed
    private void write(double p) {
        if (Math.abs(p - lastWritten) > 0.002 || (p == 0 && lastWritten != 0)) {
            motor.setPower(p);
            lastWritten = p;
        }
    }

    public boolean isOdoMode()  { return useOdo; }
    public boolean atSpeed()    { return enabled && Math.abs(targetRpm - currentRpm) < 0.05 * targetRpm; }
    public double getTargetRpm()  { return targetRpm; }
    public double getCurrentRpm() { return currentRpm; }
    public double getRawVelocity(){ return rawDegPerSec; }
    public double getPower()      { return power; }
    public double getLastDist()   { return lastDist; }
}