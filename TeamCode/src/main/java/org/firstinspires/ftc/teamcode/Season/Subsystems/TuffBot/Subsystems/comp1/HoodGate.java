package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.robot.Mechanism;

/** Hood + gate servos with velocity loss compensation. */
public class HoodGate implements Mechanism {
    private static final double GATE_OPEN   = 0;
    private static final double GATE_CLOSED = 0.25;

    private static final double HOOD_UP   = 0.35;   // dpad left
    private static final double HOOD_DOWN = 0.45;    // dpad down

    // ---- velocity compensation tuning ----
    // Works on the ratio of the RPM reading to itself, so it does not depend on the reading's units.
    private static final double HOOD_COMP_GAIN  = 4.438;   // servo units per 100% dip below the recent peak
    private static final double HOOD_COMP_MAX   = 0.1;  // max shift from the base position
    private static final double FAST_TAU        = 0.03;  // seconds, light smoothing of the RPM reading
    private static final double SLOW_TAU        = 0.6;   // seconds, how fast the baseline decays after a dip
    private static final double SPINUP_HOLD_SEC = 0.6;   // hold the base hood this long after the shooter starts
    private static final double MIN_READING     = 1.0;   // ignore readings below this (in the reading's own units)

    private Servo hood, gate;
    private double hoodPos = Double.NaN, gatePos = Double.NaN;
    private double activeBaseHood = HOOD_UP;
    private double lastComp = 0.0;

    private final ElapsedTime compTimer = new ElapsedTime();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private double fastRpm = 0, baselineRpm = 0;
    private boolean active = false;

    // debug
    private double lastDipPct = 0, peakDipPct = 0;

    public void init(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    private void setHood(double p) {
        double rounded = Math.round(p * 1000.0) / 1000.0;
        if (Double.isNaN(hoodPos) || Math.abs(rounded - hoodPos) >= 0.003) {
            hood.setPosition(rounded);
            hoodPos = rounded;
        }
    }

    private void setGate(double p) {
        double rounded = Math.round(p * 1000.0) / 1000.0;
        if (Double.isNaN(gatePos) || Math.abs(rounded - gatePos) >= 0.003) {
            gate.setPosition(rounded);
            gatePos = rounded;
        }
    }

    public Command presetUp()   { return instant(() -> { activeBaseHood = HOOD_UP; setHood(HOOD_UP); setGate(GATE_CLOSED); }); }
    public Command presetDown() { return instant(() -> { activeBaseHood = HOOD_DOWN; setGate(GATE_CLOSED); setHood(HOOD_DOWN); }); }
    public Command openGate()   { return instant(() -> setGate(GATE_OPEN)); }
    public Command closeGate()  { return instant(() -> setGate(GATE_CLOSED)); }

    /**
     * Moves the hood when the flywheel reading dips below its recent peak (e.g. a ball leaving the flywheel).
     * The baseline follows the reading up instantly and decays down slowly, so spin-up and a steady offset
     * from the target cause no movement; only a drop does. targetRpm is only used to check the shooter is on.
     * Call every loop.
     */
    /** Start-of-match positions: hood up, gate closed. */
    public void initPositions() {
        activeBaseHood = HOOD_UP;
        setHood(HOOD_UP);
        setGate(GATE_CLOSED);
    }
    public void applyVelocityCompensation(boolean isSpinning, double targetRpm, double currentRpm) {
        if (hood == null) return;

        double dt = compTimer.seconds();
        compTimer.reset();

        if (!isSpinning || targetRpm <= 0) {
            active = false;
            spinTimer.reset();
            peakDipPct = 0;
            lastDipPct = 0;
            lastComp = 0.0;
            setHood(activeBaseHood);
            return;
        }

        // Spin-up: follow the reading and hold the base position
        if (spinTimer.seconds() < SPINUP_HOLD_SEC || currentRpm < MIN_READING) {
            active = false;
            fastRpm = currentRpm;
            baselineRpm = currentRpm;
            lastDipPct = 0;
            lastComp = 0.0;
            setHood(activeBaseHood);
            return;
        }

        if (!active) {
            active = true;
            fastRpm = currentRpm;
            baselineRpm = currentRpm;
        }

        double aFast = 1.0 - Math.exp(-dt / FAST_TAU);
        double aSlow = 1.0 - Math.exp(-dt / SLOW_TAU);
        fastRpm += aFast * (currentRpm - fastRpm);

        if (fastRpm > baselineRpm) {
            baselineRpm = fastRpm;                          // rises instantly
        } else {
            baselineRpm += aSlow * (fastRpm - baselineRpm); // decays slowly
        }

        // How far the reading sits below its recent peak, as a fraction of that peak
        double dipPct = (baselineRpm - fastRpm) / Math.max(baselineRpm, MIN_READING);
        lastDipPct = dipPct;
        peakDipPct = Math.max(peakDipPct, dipPct);

        double comp = Range.clip(dipPct * HOOD_COMP_GAIN, 0.0, HOOD_COMP_MAX);
        setHood(Range.clip(activeBaseHood - comp, 0.05, 0.95));
        lastComp = -comp;
    }

    public double getHoodPosition() { return hoodPos; }
    public double getGatePosition() { return gatePos; }
    public double getLastCompensation() { return lastComp; }

    // debug getters ("reached speed" now means compensation is active)
    public boolean getReachedSpeed() { return active; }
    public double getDipPct()        { return lastDipPct; }
    public double getPeakDipPct()    { return peakDipPct; }
}