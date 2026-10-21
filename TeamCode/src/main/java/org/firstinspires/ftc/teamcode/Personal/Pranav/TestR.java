package org.firstinspires.ftc.teamcode.Personal.Pranav;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.BaseSubsystemExamples.Simplenextrobot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.OdoPID;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class TestR extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)
    ProtoBot robot;
    OdoPID pid = new OdoPID();

    Servo hood;
    Servo gate;

    public TestR(ProtoBot robot) { super(robot); this.robot = robot;}

    //Calls follower
    Follower follower;

    //Sets all of the points that the robot needs to go to
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.7758, 8, 270);
    private final Pose path1 = poseFactory.of(55.7758, 23.8906, 270);
    private final Pose point2 = poseFactory.of(40.0281, 36.5897, 180);
    private final Pose point2Control1 = poseFactory.of(41.6965, 25.5388, 0);
    private final Pose point3 = poseFactory.of(18.1744, 36.4723, 180);
    private final Pose point4 = poseFactory.of(28.5915, 37.0344, 180);
    private final Pose point5 = poseFactory.of(54.3518, 108.225, 90);
    private final Pose point5Control1 = poseFactory.of(25.2278, 113.2785, 0);

    public Path path1() {
        return line(start, path1).constant(path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).constant(point3);
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    //Command spause = waitMs(1000); // waits 500ms
    Command intpause() { return waitMs(1000); }
    Command warmup()   { return waitMs(1775); }
    Command quick()    { return waitMs(775); }
    Command spause()   { return waitMs(2000); }
    Command gpause()   { return waitMs(1000); }

    Command hoodboi() {
        return instant(() -> hood.setPosition(0.3));
    }

    Command gateopen() {
        return instant(() -> gate.setPosition(0));
    }

    Command gateclose() {
        return instant(() -> gate.setPosition(0.1));
    }

    //Sequentially calls all of the paths
    private Command autoRoutine() {
        return sequential(
                hoodboi(),
                gateopen(),
                gpause(),
                pid.run(),
                warmup(),
                robot.intake.run(),
                robot.transfer.run(),
                spause(),
                pid.stop(),
                robot.intake.stop(),
                robot.transfer.stop(),
                gateclose(),
                follow(follower, path1()),
                robot.intake.run(),
                robot.transfer.slow(),
                follow(follower, path2()),
                follow(follower, path3()),
                intpause(),
                robot.intake.stop(),
                robot.transfer.stop(),
                follow(follower, path4()),
                hoodboi(),
                gateopen(),
                gpause(),
                pid.run(),
                follow(follower,path5()),
                quick(),
                robot.intake.run(),
                robot.transfer.run(),
                spause(),
                pid.stop(),
                robot.transfer.stop(),
                robot.intake.stop(),
                gateclose()
        );
    }

    private boolean initialized = false;

    //Runs periodically in init
    @Override
    public void disabledPeriodic() {
        Telemetry.log("Status", "Init");
        if (initialized) return;
        initialized = true;

        Scheduler.reset();
        hood = hardwareMap.get(Servo.class, "hood");

        gate = hardwareMap.get(Servo.class,"gate");

        pid.init(hardwareMap);            // original signature
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        pid.setFollower(follower);        // only needed if you use OdoRun()
    }

    //Runs once at start
    @Override
    public void start() {
        Telemetry.log("Status", "Start");

        //Makes the auto start when play button is pressed
        schedule(autoRoutine());
    }

    //Runs periodically through the game
    @Override
    public void periodic() {
        Telemetry.log("Status", "Running");

        //Starts follower and auto path
        follower.update();
        pid.update();                     // required, because the Commands only enable the PID
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    //Runs once at stop
    @Override
    public void end() {
        Telemetry.log("Status", "End");
    }
}
