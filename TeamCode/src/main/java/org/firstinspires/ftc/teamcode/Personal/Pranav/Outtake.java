package org.firstinspires.ftc.teamcode.Personal.Pranav;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import static dev.nextftc.units.Units.DegreesPerSecond;

import com.pedropathing.ivy.Command;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;

public class Outtake {
    NextMotor motor = new NextMotor("outtake");


    public Command run() {
        motor.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        // Target speed in RPM — adjust to your motor
        double targetRPM = 500.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s)
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint

        return infinite(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));

    }

    public Command stop() { return instant(() -> motor.setThrottle(0)); }
}
