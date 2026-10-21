package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;

public class OdoPID {

    // ---- Shooter PID constants (same as MecanumTeleOp) ----
    private static final double kS = 0.12;     // power to overcome friction
    private static final double kV = 0.001;    // power per RPM
    private static final double kP = 0.025;     // error is in deg/s
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double DEG_PER_SEC_PER_RPM = 6.0; // RPM x 6 = deg/s
    private static final double MAX_INTEGRAL = 0.3;        // anti-windup limit
    private static final double SHOOTER_SIGN = -1;         // reverses final motor power only

    private double targetRpm = 25;   // same default as the TeleOp
    private boolean enabled = false;
    private boolean useOdo = false;

    Follower follower;
    double integral = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    DcMotorEx outtakeLeft;
    public String alliance = "EMPTY";

    // for telemetry
    private double currentRpm = 0, rawDegPerSec = 0, shooterPower = 0, lastDist = 0;

    public void init(HardwareMap hardwareMap) {
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft"); // sets the FIELD
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower = Constants.create(hardwareMap);
    }

    // Optional: share TestR's follower instead of the one made in init()
    public void setFollower(Follower f) { this.follower = f; }

    public void reset() {
        integral = 0;
        lastError = 0;
        enabled = false;
        shooterPower = 0;
        if (outtakeLeft != null) outtakeLeft.setPower(0);
    }

    // ---- Commands: they only turn the PID on/off; update() does the work ----
    public Command run() {
        return instant(() -> start(false));
    }

    public Command OdoRun() {
        return instant(() -> start(true));
    }

    public Command stop() {
        return instant(this::reset);
    }

    private void start(boolean odo) {
        useOdo = odo;
        enabled = true;
        integral = 0;
        lastError = 0;
        timer.reset();
    }

    // ---- Call every loop from periodic() ----
    public void update() {
        if (outtakeLeft == null) return;

        // Not shooting: reset controller state and cut power (same as TeleOp's else branch)
        if (!enabled) {
            integral = 0;
            lastError = 0;
            shooterPower = 0;
            outtakeLeft.setPower(0);
            return;
        }

        if (useOdo) {
            Pose p = follower.pose();
            double x = p.x(), y = p.y(), xt = 0, yt = 0;
            if ("BLUE".equals(alliance)) { xt = 84; yt = (y >= 70.75) ? 82 : 56; }
            else if ("RED".equals(alliance)) { xt = 58; yt = (y >= 70.75) ? 82 : 56; }
            lastDist = Math.hypot(xt - x, yt - y);
            targetRpm = 0.84375 * lastDist;
        }

        // ---------------- Shooter PID (copied from MecanumTeleOp) ----------------
        double dt = timer.seconds();
        timer.reset();

        rawDegPerSec = outtakeLeft.getVelocity(AngleUnit.DEGREES);
        // Magnitude so the PID always sees a positive speed; the sign is applied at the end
        double currentDegPerSec = Math.abs(rawDegPerSec);
        currentRpm = currentDegPerSec / DEG_PER_SEC_PER_RPM;

        double targetDegPerSec = targetRpm * DEG_PER_SEC_PER_RPM;
        double error = targetDegPerSec - currentDegPerSec;

        // Integral with anti-windup
        if (dt > 0) {
            integral += error * dt * kI;
            integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double feedforward = kS + kV * targetRpm;
        shooterPower = feedforward + kP * error + integral + kD * derivative;
        shooterPower = Range.clip(shooterPower, 0, 1);

        outtakeLeft.setPower(SHOOTER_SIGN * shooterPower);
    }

    // Getters so TestR can print telemetry (the Blocks telemetry import is null in normal opmodes)
    public double getTargetRpm()  { return targetRpm; }
    public double getCurrentRpm() { return currentRpm; }
    public double getRawVelocity(){ return rawDegPerSec; }
    public double getPower()      { return shooterPower; }
    public double getLastDist()   { return lastDist; }
}