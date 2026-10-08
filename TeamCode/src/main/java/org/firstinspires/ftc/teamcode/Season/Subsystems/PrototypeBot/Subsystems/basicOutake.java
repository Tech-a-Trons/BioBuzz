package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class basicOutake implements Mechanism {

    private final NextMotor outakeleft = new NextMotor("outakeleft");
    private final NextMotor outakeright = new NextMotor("outakeright");

    private void setBoth(double power) {
        outakeleft.setThrottle(power);
        outakeright.setThrottle(power);
    }

    public Command run()     { return infinite(() -> setBoth(1)); }
    public Command mid()     { return infinite(() -> setBoth(0.5)); }
    public Command slow()    { return infinite(() -> setBoth(0.25)); }
    public Command reverse() { return infinite(() -> setBoth(-1)); }

    public Command stop()    { return instant(() -> setBoth(0)); }
}