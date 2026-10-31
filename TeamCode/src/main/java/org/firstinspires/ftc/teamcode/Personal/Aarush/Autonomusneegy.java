package org.firstinspires.ftc.teamcode.Personal.Aarush;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static java.util.concurrent.locks.LockSupport.park;

import org.firstinspires.ftc.teamcode.Season.Auto.pedro.Constants;

public class Autonomusneegy extends OpMode {
    @Override
    public void start() {
        schedule(follow(follower, path1()));
    }
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8.5463, 90);
    private final Pose point1Start = poseFactory.of(56, 8.5463, 180);
    private final Pose point1 = poseFactory.of(10.0647, -3, 180);
    private final Pose point2 = poseFactory.of(58.9315, 116.8427, 90);
    private final Pose point2Control1 = poseFactory.of(58.0193, 11.7181, 0);
    private final Pose point2Control2 = poseFactory.of(69.917, 26.3465, 0);
    private final Pose point3 = poseFactory.of(48.473, 122.7558, 90);
    private final Pose point3Control1 = poseFactory.of(44.1805, 114.6525, 0);
    private final Pose point4 = poseFactory.of(55.1795, 8.4585, 270);
    private final Pose park = poseFactory.of(2.4585, 67.5232, 0);

    public Path path1() {
        return line(point1Start, point1).linear(point1Start, point1);
    }

    public Path path2() {
        return curve(point1, point2Control1, point2Control2, point2).linear(point1, point2);
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).constant(point3);
    }

    public Path path4() {
        return curve(point3, park, point4).linear(point3, point4);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
