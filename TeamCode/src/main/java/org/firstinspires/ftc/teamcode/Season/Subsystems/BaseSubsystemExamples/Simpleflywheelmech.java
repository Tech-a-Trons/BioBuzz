package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static dev.nextftc.units.Units.DegreesPerSecond;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

//Mechanism -> New subsystem
public class Simpleflywheelmech implements Mechanism {

    //Initialize the motor
    NextMotor motor = new NextMotor("outtake");

    public Command run() {

        //Sets up the PID for the motor
        motor.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        // Target RPM of 50
        double targetRPM = 50.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s) -> (50 * 6) = 300 degrees
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint to 300 degrees
        return infinite(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));

        //In case that infinite doesn't work, use this return!
        //return instant(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));
    }

    public Command stop() {

        //Sets up the PID for the motor
        motor.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        //Target RPM = 0, more gradual spindown
        double targetRPM = 0.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s)
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint to O
        return instant(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));

        //In case that instant doesn't work, use this return!
        //return infinite(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));
    }
}
