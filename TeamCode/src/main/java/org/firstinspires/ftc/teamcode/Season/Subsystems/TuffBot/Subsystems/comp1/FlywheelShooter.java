package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    private static final double kS = 0.1;
    private static final double kV = 0.000027;
    private static final double kP = 0.00007;
    private static final double kI = 0.0000;
    private static final double kD = 0.0;

    // Goal positions, field coords (inches), (144,144) = top-right
    private static final double[][] GOALS = {
            {57.26, 59.94},  // bottom-left
            {57.65, 86.69},  // top-left
            {83.53, 83.10},  // top-right
            {83.62, 52.67}   // bottom-right
    };
    private static final double FIELD_MID = 72.0;

    // rpm = RPM_PER_INCH * distanceToGoal + RPM_AT_ZERO (fit from 1010 @ 51.95", 970 @ 36.94")
    private static final double RPM_PER_INCH = 2.66667;
    private static final double RPM_AT_ZERO = 871.49;
    private static final double MIN_RPM = 850;   // placeholder - set to your safe limits
    private static final double MAX_RPM = 1100;

    private static double TARGET_RPM = 1010; //1010 for straight aligned to the back //970 when shooting one tile forward from wall
    private static final double DEG_PER_SEC_PER_RPM = 6.0;
    private static final double MAX_INTEGRAL = 0;

    // Multiplies the encoder's deg/sec reading so it matches the real flywheel speed.
    // Calibrate: set kP = 0, hold the trigger until it settles, read Current RPM,
    // then VELOCITY_SCALE = 1380 / (that reading). Restore kP afterwards.
    private static final double VELOCITY_SCALE = 1.0;

    // Direction of each motor. If the right motor fights the left, flip RIGHT_SIGN.
    private static final double LEFT_SIGN = -1;
    private static final double RIGHT_SIGN = 1;

    private DcMotorEx motorLeft;   // commanded + encoder
    private DcMotorEx motorRight;  // follower
    private Servo light;
    private Follower follower;
    private GoalSelector goal;

    private final ElapsedTime timer = new ElapsedTime();
    private double integral = 0, lastError = 0;
    private boolean enabled = false;
    private double lastWritten = 2;
    Pose pose;

    private double targetRpm = TARGET_RPM, currentRpm = 0, rawDegPerSec = 0, power = 0;

    public void init(HardwareMap hardwareMap, Follower follower, GoalSelector goal) {
        this.follower = follower;
        this.goal = goal;

        motorLeft = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        follower.update();

        pose = follower.pose();

//        light = hardwareMap.get(Servo.class, "light");
//        light.setPosition(0);
        // Right motor never uses its encoder; it only mirrors the left's power.
        motorRight = hardwareMap.get(DcMotorEx.class, "outtakeright");

        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Command setSpinning(boolean on) {
        return instant(() -> {
            if (on && !enabled) {
                integral = 0;
                lastError = 0;
                timer.reset();
            }
            enabled = on;
        });
    }

    /** Distance (in) from the robot to the goal in its own quadrant. */
    private double distanceToGoal(Pose p) {
        int idx;
        if (p.x() < FIELD_MID) idx = (p.y() < FIELD_MID) ? 0 : 1;  // left half
        else                   idx = (p.y() < FIELD_MID) ? 3 : 2;  // right half
        return Math.hypot(GOALS[idx][0] - p.x(), GOALS[idx][1] - p.y());
    }

//    @Override
//    public void periodic() {
//        Mechanism.super.periodic();
//        update();
//    }

    // FlywheelShooter.java
    private ShooterLeds leds;
    public void setLeds(ShooterLeds leds) { this.leds = leds; }

    @Override
    public void periodic() {
        Mechanism.super.periodic();
        update();
        if (leds != null) leds.update();
    }

    private void update() {
        follower.update();

        if (motorLeft == null || motorRight == null) return;
        if (!enabled) {
            integral = 0;
            lastError = 0;
            power = 0;
            targetRpm = TARGET_RPM;
            write(0);
            return;
        }

        // in update(), replacing the TARGET_RPM line
        pose = follower.pose();   // refresh every loop (it was only set once in init before)
        TARGET_RPM = Range.clip(RPM_PER_INCH * distanceToGoal(pose) + RPM_AT_ZERO, MIN_RPM, MAX_RPM);
        targetRpm = TARGET_RPM;

        double dt = timer.seconds();
        timer.reset();

        // Only the left motor's encoder is used for feedback
        rawDegPerSec = motorLeft.getVelocity(AngleUnit.DEGREES) * VELOCITY_SCALE;
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

//        if (errorDegPerSec <=100 && errorDegPerSec >= 0) {
//            light.setPosition(0.5);
//        }

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