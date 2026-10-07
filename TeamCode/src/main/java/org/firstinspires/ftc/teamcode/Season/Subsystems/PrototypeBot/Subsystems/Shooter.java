package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static dev.nextftc.units.Units.DegreesPerSecond;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {

    NextMotor left = new NextMotor("outtakeleft");
//    NextMotor right = new NextMotor("outtakeright");

    public Shooter() {
        // Right motor mimics the left. Use REVERSE if the motors are mounted
        // facing opposite directions (typical for two-sided flywheels),
        // or Direction.FORWARD / no argument if they spin the same way.
//        right.follow(left, NextMotor.Direction.REVERSE);
//REVERSE
        // PID constants only need to be set once, not every time a command is built
        left.getVelocityConstants()
                .withP(0.02)
                .withI(0.0)
                .withD(0.0);
    }
    private static final double kS = 0.12;      // power to overcome friction
    private static final double kV = 0.0074;

    public Command run(double rpm) {
        double targetDegPerSec = rpm * 6.0; // RPM x 6 = deg/s
        return infinite(() -> left.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));
    }

    public Command stop() {
        return instant(() -> left.setVelocitySetpoint(DegreesPerSecond.of(0)));
    }
}