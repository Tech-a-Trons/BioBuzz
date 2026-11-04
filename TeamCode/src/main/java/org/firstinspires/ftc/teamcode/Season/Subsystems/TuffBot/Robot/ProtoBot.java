package org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Season.SensorStuff.Limelight.hiveTip;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.Drivetrain;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.EasyTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.FlywheelShooter;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.HoodGate;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.TuffBot.Subsystems.comp1.GoalSelector;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class ProtoBot implements NextRobot {

    // All of the mechanisms
    public final Drivetrain drive = new Drivetrain();
    public final IntakeTransfer intakeTransfer = new IntakeTransfer();
    public final HoodGate hoodGate = new HoodGate();
    public final GoalSelector goal = new GoalSelector();
    public final FlywheelShooter shooter = new FlywheelShooter();
    public final hiveTip tip = new hiveTip();

    // EasyTurret is a plain class (not a Mechanism), so it's a field but not in getMechanisms()
    public final EasyTurret turret = new EasyTurret();

    /**
     * The new subsystems grab their hardware in init(hardwareMap), so call this once
     * from the OpMode instead of relying on the framework to do it.
     */
    public void init(HardwareMap hardwareMap, Follower follower, String alliance) {
        goal.setAlliance(alliance);
        drive.init(hardwareMap);
        intakeTransfer.init(hardwareMap);
        hoodGate.init(hardwareMap);
        shooter.init(hardwareMap, follower, goal);
        turret.init(hardwareMap, follower, goal);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drive, intakeTransfer, hoodGate, shooter, tip);
    }
}