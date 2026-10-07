package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
@Disabled
public class Simplenextrobot implements NextRobot {

    //Calls all of the mechanisms that need to be used on the robot
    public Simpleflywheelmech flywheel = new Simpleflywheelmech();
    public Simplemotormech motor = new Simplemotormech();
    public SimpleServoMech servo = new SimpleServoMech();
    public Simpledrivetrainmech drivetrain = new Simpledrivetrainmech();

    //Essentially defines each mechanism as part of the robot, can be called in other programs
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(servo,motor,flywheel, drivetrain);
    }
}
