// subsystems/Intake.java
package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
public class Intake implements Mechanism {

    private final NextMotor motor = new NextMotor("motor");

    public Command stop()              { return instant(() -> motor.setThrottle(0)); }
    public Command quarterPower()      { return instant(() -> motor.setThrottle(0.25)); }
    public Command halfPower()         { return instant(() -> motor.setThrottle(0.5)); }
    public Command threeQuarterPower() { return instant(() -> motor.setThrottle(0.75)); }
}