package org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems;

import static dev.nextftc.units.Units.Degrees;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    NextMotor motor = new NextMotor("turretMotor", Degrees.of(360.0 / (537.7 * 3.0)));

    private Supplier<Pose> pose;
    private double commandedDeg = 0;

    public Turret(Supplier<Pose> pose) {
        this.pose = pose;
        motor.getPositionConstants().withP(0.1).withD(0.0);
    }

    public static double xt = 60;
    public static double yt = 60;
    public static double offsetDeg = 0;

    @Override
    public Command getDefaultCommand() {
        return infinite(this::track);
    }

    public Command manual(DoubleSupplier power) {
        return infinite(() -> motor.setThrottle(power.getAsDouble()));
    }

    public void setTarget(double x, double y) {
        xt = x;
        yt = y;
    }

    private void track() {
        Pose p = pose.get();
        if (p == null) return;

        double fieldDeg = Math.toDegrees(Math.atan2(yt - p.y(), xt - p.x()));
        double desired = wrap(fieldDeg - Math.toDegrees(p.heading()) + offsetDeg);

        commandedDeg = closestLegal(desired, commandedDeg);
        motor.setPositionSetpoint(Degrees.of(commandedDeg));
    }

    private static double wrap(double a) {
        a %= 360;
        if (a > 180) a -= 360;
        else if (a <= -180) a += 360;
        return a;
    }

    private static double closestLegal(double desired, double current) {
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double c = desired + 360 * k;
            if (c < -180 || c > 180) continue;
            if (Double.isNaN(best) || Math.abs(c - current) < Math.abs(best - current)) best = c;
        }
        return Double.isNaN(best) ? Math.max(-180, Math.min(180, desired)) : best;
    }
}
