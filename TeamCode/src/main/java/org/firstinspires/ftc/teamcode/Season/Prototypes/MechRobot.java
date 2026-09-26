package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
public class MechRobot implements NextRobot {

    //Calls all of the mechanisms that need to be used on the robot
    public MechDrivetrain drivetrain = new MechDrivetrain();

    //Essentially defines each mechanism as part of the robot, can be called in other programs
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain);}

}
