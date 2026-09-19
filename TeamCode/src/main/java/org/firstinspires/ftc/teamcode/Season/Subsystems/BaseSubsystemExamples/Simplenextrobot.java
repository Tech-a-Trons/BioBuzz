package org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Simplenextrobot implements NextRobot {

    //Calls all of the mechanisms
    public Simpleflywheelmech flywheel = new Simpleflywheelmech();
    public Simplemotormech motor = new Simplemotormech();
    public SimpleServoMech servo = new SimpleServoMech();

    //Essentially defines each mechanism as part of the robot, can be call in other programs
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(servo,motor,flywheel);
    }
}
