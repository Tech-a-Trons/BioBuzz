package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

public class ShooterLeds implements Mechanism {
    private static final double TOLERANCE = 0.15;      // leave; harmless, never fires at ~35
    private static final double SETTLE_BAND_RPM = 6;   // was 25; at a ~35 reading, 25 is huge
    private static final long SETTLE_MS = 300;
    private static final double MIN_RPM = 15;          // was 50; must be below your ~35 plateau

    private static final RevBlinkinLedDriver.BlinkinPattern NOT_READY_PATTERN =
            RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private static final RevBlinkinLedDriver.BlinkinPattern READY_PATTERN =
            RevBlinkinLedDriver.BlinkinPattern.GREEN;

    private RevBlinkinLedDriver blinkin;
    private FlywheelShooter shooter;
    private RevBlinkinLedDriver.BlinkinPattern lastPattern = null;

    private double anchorRpm = 0;
    private long anchorTimeMs = 0;

    public void init(HardwareMap hardwareMap, FlywheelShooter shooter) {
        this.shooter = shooter;
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setPattern(NOT_READY_PATTERN);
    }

    @Override
    public void periodic() {
        Mechanism.super.periodic();
        update();
    }

    public void update() {
        if (blinkin == null || shooter == null) return;

        long now = System.currentTimeMillis();

        if (!shooter.isSpinning()) {
            anchorRpm = 0;
            anchorTimeMs = now;
            setPattern(NOT_READY_PATTERN);
            return;
        }

        double rpm = shooter.getCurrentRpm();
        double target = shooter.getTargetRpm();

        // Restart the settle clock whenever speed moves outside the band
        if (Math.abs(rpm - anchorRpm) > SETTLE_BAND_RPM) {
            anchorRpm = rpm;
            anchorTimeMs = now;
        }

        boolean nearTarget = Math.abs(target - rpm) <= TOLERANCE * target;
        boolean settled = rpm > MIN_RPM && (now - anchorTimeMs) >= SETTLE_MS;

        setPattern((nearTarget || settled) ? READY_PATTERN : NOT_READY_PATTERN);
    }

    private void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != lastPattern) {
            blinkin.setPattern(pattern);
            lastPattern = pattern;
        }
    }
}