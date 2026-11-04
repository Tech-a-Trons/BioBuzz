package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.pedropathing.math.Pose;

/**
 * Static hand-off from Auto to TeleOp. At the end of auto: PoseStorage.pose = follower.pose();
 * Static values survive between opmodes as long as the robot app isn't restarted.
 */
public class PoseStorage {
    public static Pose pose = null;
}