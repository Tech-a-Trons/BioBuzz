package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

/** Two ramp servos moving in unison. Only sends a position when it changes. */
public class Ramp implements Mechanism {
    // Fill these in using RampTuner
    public static final double RAMP_ZERO = 0.0;
    private static final double RAMP_UP   = 0.1;  // TODO: tune
    private static final double RAMP_DOWN = 0.42;  // TODO: tune

    private Servo rampRight, rampLeft;
    private double rampPos = Double.NaN;

    public void init(HardwareMap hardwareMap) {
        rampRight = hardwareMap.get(Servo.class, "rampright");
        rampLeft  = hardwareMap.get(Servo.class, "rampleft");

        // They face opposite directions, so reverse ONE of them.
        // That way the same position number moves both the same way physically.
        // If the ramp fights itself, move this reverse to rampRight instead.
        rampLeft.setDirection(Servo.Direction.REVERSE);
    }

    /** Sets both servos immediately (also usable outside of commands). */
    public void setRamp(double p) {
        if (p != rampPos) {
            rampRight.setPosition(p);
            rampLeft.setPosition(p);
            rampPos = p;
        }
    }

    public Command zero() { return instant(() -> setRamp(RAMP_ZERO)); }
    public Command up()   { return instant(() -> setRamp(RAMP_UP)); }
    public Command down() { return instant(() -> setRamp(RAMP_DOWN)); }

    /** Last commanded position (no hardware call). */
    public double getRampPosition() { return rampPos; }
}