package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static dev.nextftc.units.Units.DegreesPerSecond;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class ProtoOuttake implements Mechanism { //Initialize the motor
    NextMotor motor = new NextMotor("omotor1");
    NextMotor motor2 = new NextMotor("omotor2");
    NextServo servo = new NextServo("oservo1");
    NextServo servo2 = new NextServo("oservo2");

    public Command run() {

        //Sets up the PID for the motor
        motor.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        // Target RPM of 50
        double targetRPM = 100.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s) -> (50 * 6) = 300 degrees
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint to 300 degrees
        return infinite(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));

        //In case that infinite doesn't work, use this return!
        //return instant(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));
    }

    public Command run2() {

        //Sets up the PID for the motor

        motor2.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        // Target RPM of 50
        double targetRPM = 100.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s) -> (50 * 6) = 300 degrees
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint to 300 degrees
        return infinite(() -> motor2.setVelocitySetpoint(DegreesPerSecond.of(-targetDegPerSec)));

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

    public Command stop2() {

        //Sets up the PID for the motor
        motor2.getVelocityConstants()
                .withP(0.002)
                .withI(0.0)
                .withD(0.0);

        //Target RPM = 0, more gradual spindown
        double targetRPM = 0.0;

        // Convert RPM to degrees per second (RPM × 6 = deg/s)
        double targetDegPerSec = targetRPM * 6.0;

        // Set the velocity setpoint to O
        return instant(() -> motor2.setVelocitySetpoint(DegreesPerSecond.of(-targetDegPerSec)));

        //In case that instant doesn't work, use this return!
        //return infinite(() -> motor.setVelocitySetpoint(DegreesPerSecond.of(targetDegPerSec)));
    }

    public Command servo1N() {
        return infinite(() -> servo.setPosition(0.3));
    }

    public Command servo2N() {
        return infinite(() -> servo.setPosition(-0.3));
    }

    public Command servo1P() {
        return infinite(() -> servo.setPosition(0));
    }

    public Command servo2P() {
        return infinite(() -> servo.setPosition(0));
    }
}

