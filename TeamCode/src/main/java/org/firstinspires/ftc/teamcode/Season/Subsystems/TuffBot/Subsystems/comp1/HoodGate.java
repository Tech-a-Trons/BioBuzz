package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

/** Hood + gate servos. Only sends a position when it changes. */
public class HoodGate implements Mechanism {
    private static final double GATE_OPEN   = 0.05;
    private static final double GATE_CLOSED = 0.25;

    // Hood presets (from the simple TurretOp)
    private static final double HOOD_UP   = 0.0;   // dpad up
    private static final double HOOD_DOWN = 0.0;  // dpad down

    private Servo hood, gate;
    private double hoodPos = Double.NaN, gatePos = Double.NaN;

    public void init(HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    private void setHood(double p) { if (p != hoodPos) { hood.setPosition(p); hoodPos = p; } }
    private void setGate(double p) { if (p != gatePos) { gate.setPosition(p); gatePos = p; } }

    public Command presetUp()   { return instant(() -> { setHood(HOOD_UP);   setGate(GATE_OPEN); }); }
    public Command presetDown() { return instant(() -> { setGate(GATE_OPEN); setHood(HOOD_DOWN);  }); }
    public Command openGate()   { return instant(() -> setGate(GATE_OPEN)); }
    public Command closeGate()  { return instant(() -> setGate(GATE_CLOSED)); }

    /** Last commanded gate position (no hardware call). */
    public double getGatePosition() { return gatePos; }
}