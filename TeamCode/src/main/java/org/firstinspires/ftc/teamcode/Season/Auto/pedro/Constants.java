package org.firstinspires.ftc.teamcode.Season.Auto.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("bl");
                c.frontRightName.set("fr");
                c.backRightName.set("br");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
//    public static Follower create(HardwareMap h) {
//        return new Follower(
//                new PinpointLocalizer(h, localizerConfig),
//                new Mecanum(h, drivetrainConfig),
//                new Foresight(foresightConfig)
//        );
//    }
}