package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.robot.Mechanism;

/** Hood + gate servos with velocity loss compensation. */
public class HoodGate implements Mechanism {
    private static final double GATE_OPEN   = 0;
    private static final double GATE_CLOSED = 0.3;

    private static final double HOOD_UP   = 0.4;   // dpad up
    private static final double HOOD_DOWN = 0.22;  // dpad down

    private static final double HOOD_COMP_GAIN = 0.15;   // Hood adjustment for velocity loss percentage drop

    private Servo hood, gate;
    private double hoodPos = Double.NaN, gatePos = Double.NaN;
    private double activeBaseHood = HOOD_UP;
    private double lastComp = 0.0;

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

    public Command presetUp()   { return instant(() -> { activeBaseHood = HOOD_UP; setHood(HOOD_UP); setGate(GATE_OPEN); }); }
    public Command presetDown() { return instant(() -> { activeBaseHood = HOOD_DOWN; setGate(GATE_OPEN); setHood(HOOD_DOWN); }); }
    public Command openGate()   { return instant(() -> setGate(GATE_OPEN)); }
    public Command closeGate()  { return instant(() -> setGate(GATE_CLOSED)); }

    /**
     * Compensates hood position for flywheel velocity loss (voltage sag / recovery lag).
     */
    public void applyVelocityCompensation(boolean isSpinning, double targetRpm, double currentRpm) {
        if (hood == null) return;

        if (!isSpinning || targetRpm <= 0) {
            setHood(activeBaseHood);
            lastComp = 0.0;
            return;
        }

        // Velocity loss compensation (percentage RPM drop)
        double rpmDropPct = (targetRpm - currentRpm) / targetRpm;

        if (rpmDropPct > 0.02) {
            double comp = Range.clip(rpmDropPct * HOOD_COMP_GAIN, 0.0, 0.08);
            double compensatedPos = Range.clip(activeBaseHood - comp, 0.05, 0.95);
            setHood(compensatedPos);
            lastComp = -comp;
        } else {
            setHood(activeBaseHood);
            lastComp = 0.0;
        }
    }

    public double getGatePosition() { return gatePos; }
    public double getLastCompensation() { return lastComp; }
}
