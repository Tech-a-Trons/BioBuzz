package org.firstinspires.ftc.teamcode.Season.Auto;

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
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Robot.ProtoBot;
import org.firstinspires.ftc.teamcode.Season.Subsystems.PrototypeBot.Subsystems.OdoPID;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class TestBlueR extends NextOpMode {

    //Calls the mechanisms "part" of the robot (See Simplenextrobot for more detail)
    ProtoBot robot;
    OdoPID pid = new OdoPID();

    Servo hood;
    Servo gate;

    public TestBlueR(ProtoBot robot) { super(robot); this.robot = robot;}

    //Calls follower
    Follower follower;

    //Sets all of the mirrored points that the robot needs to go to (Y: 144 - y, Heading: 360 - heading)
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.7758, 136.0, 90);
    private final Pose path1 = poseFactory.of(55.7758, 120.1094, 90);
    private final Pose point2 = poseFactory.of(40.0281, 107.4103, 180);
    private final Pose point2Control1 = poseFactory.of(41.6965, 118.4612, 0);
    private final Pose point3 = poseFactory.of(18.1744, 107.5277, 180);
    private final Pose point4 = poseFactory.of(28.5915, 106.9656, 180);
    private final Pose point5 = poseFactory.of(54.3518, 35.775, 270);
    private final Pose point5Control1 = poseFactory.of(25.2278, 30.7215, 0);
    private final Pose point6 = poseFactory.of(48.9558, 30.8617, 270);
    private final Pose point7 = poseFactory.of(48.8392, 23.5428, 270);
    private final Pose point8 = poseFactory.of(48.6531, 30.5337, 270);
    private final Pose point9 = poseFactory.of(54.309, 35.9087, 270);
    private final Pose point10 = poseFactory.of(55.6124, 127.8764, 180);
    private final Pose point10Control1 = poseFactory.of(77.8076, 68.9242, 0);
    private final Pose point11 = poseFactory.of(18.2275, 127.7662, 180);
    private final Pose point12 = poseFactory.of(55.7444, 127.861, 180);
    private final Pose point13 = poseFactory.of(56.2591, 120.5955, 90);

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

    public Path path6() {
        return line(point5, point6).constant(point6);
    }

    public Path path7() {
        return line(point6, point7).constant(point7);
    }

    public Path path8() {
        return line(point7, point8).constant(point8);
    }

    public Path path9() {
        return line(point8, point9).constant(point9);
    }

    public Path path10() {
        return curve(point9, point10Control1, point10).linear(point9, point10);
    }

    public Path path11() {
        return line(point10, point11).constant(point11);
    }

    public Path path12() {
        return line(point11, point12).constant(point12);
    }

    public Path path13() {
        return line(point12, point13).linear(point12, point13);
    }

    //Command spause = waitMs(1000); // waits 500ms
    Command intpause() { return waitMs(1000); }
    Command warmup()   { return waitMs(1775); }
    Command quick()    { return waitMs(775); }
    Command spause()   { return waitMs(2000); }
    Command gpause()   { return waitMs(500); }

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
//                hoodboi(),
//                gateopen(),
//                gpause(),
//                pid.run(),
//                warmup(),
//                robot.intake.run(),
//                robot.transfer.run(),
//                spause(),
//                pid.stop(),
//                robot.intake.stop(),
//                robot.transfer.stop(),
//                gateclose(),
                follow(follower, path1()),
//                robot.intake.run(),
//                robot.transfer.slow(),
                follow(follower, path2()),
                follow(follower, path3()),
//                intpause(),
//                robot.intake.stop(),
//                robot.transfer.stop(),
//                follow(follower, path4()),
//                hoodboi(),
//                gateopen(),
//                gpause(),
//                pid.run(),
                follow(follower,path5()),
//                quick(),
//                robot.intake.run(),
//                robot.transfer.run(),
//                spause(),
//                pid.stop(),
//                robot.transfer.stop(),
//                robot.intake.stop(),
//                gateclose(),
//                robot.intake.run(),
//                robot.transfer.slow(),
                follow(follower,path6()),
                follow(follower,path7()),
                //intpause(),
                follow(follower,path8()),
//                robot.intake.stop(),
//                robot.transfer.stop(),
//                pid.run(),
                follow(follower,path9())//,
//                hoodboi(),
//                gateopen(),
//                gpause(),
//                robot.transfer.run(),
//                robot.intake.run(),
//                spause()

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
